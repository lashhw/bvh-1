#ifndef BVH_BVH_HPP
#define BVH_BVH_HPP

struct Bvh {
    struct Node {
        BoundingBox bbox;
        int num_primitives;
        union {
            int left_node_index;  // used when node != leaf
            int first_primitive_index;  // used when node == leaf
        };

        bool is_leaf() const { return num_primitives != 0; }
    };

    Bvh(std::vector<Triangle> *triangles_ptr) : triangles_ptr(triangles_ptr) {
        // allocate temporary memory for BVH construction
        int num_primitives = triangles_ptr->size();
        std::vector<BoundingBox> bboxes(num_primitives);
        std::vector<Vec3> centers(num_primitives);
        std::vector<float> costs(num_primitives);
        std::vector<bool> marks(num_primitives);
        std::array<std::vector<int>, 3> sorted_references;
        sorted_references[0].resize(num_primitives);
        sorted_references[1].resize(num_primitives);
        sorted_references[2].resize(num_primitives);

        num_nodes = 1;
        nodes.resize(2 * num_primitives);

        for (int i = 0; i < num_primitives; i++) {
            bboxes[i] = (*triangles_ptr)[i].bounding_box();
            nodes[0].bbox.extend(bboxes[i]);
            centers[i] = (*triangles_ptr)[i].center();
        }

        for (int axis = 0; axis < 3; axis++) {
            std::iota(sorted_references[axis].begin(), sorted_references[axis].end(), 0);
            std::sort(sorted_references[axis].begin(), sorted_references[axis].end(),
                      [&](int i, int j) { return centers[i][axis] < centers[j][axis]; });
        }

        // build BVH
        std::stack<std::array<int, 4>> stack;  // node_index, begin, end, depth
        int node_index = 0;
        int begin = 0;
        int end = num_primitives;
        int depth = 0;
        auto check_and_update_and_pop_stack = [&]() -> bool {
            if (stack.empty()) return false;
            node_index = stack.top()[0];
            begin = stack.top()[1];
            end = stack.top()[2];
            depth = stack.top()[3];
            stack.pop();
            return true;
        };

        while (true) {
            Node &curr_node = nodes[node_index];
            int curr_num_primitives = end - begin;

            if (curr_num_primitives <= 1 || depth >= MAX_DEPTH) {
                curr_node.num_primitives = curr_num_primitives;
                curr_node.first_primitive_index = begin;

                if (check_and_update_and_pop_stack()) continue;
                else break;
            }

            float best_cost = FLT_MAX;
            int best_axis = -1;
            int best_split_index = -1;

            for (int axis = 0; axis < 3; axis++) {
                BoundingBox tmp_bbox;
                for (int i = end - 1; i > begin; i--) {
                    tmp_bbox.extend(bboxes[sorted_references[axis][i]]);
                    costs[i] = tmp_bbox.half_area() * (end - i);
                }

                tmp_bbox = BoundingBox();
                for (int i = begin; i < end - 1; i++) {
                    tmp_bbox.extend(bboxes[sorted_references[axis][i]]);
                    float cost = tmp_bbox.half_area() * (i + 1 - begin) + costs[i + 1];
                    if (cost < best_cost) {
                        best_cost = cost;
                        best_axis = axis;
                        best_split_index = i + 1;
                    }
                }
            }

            float max_split_cost = curr_node.bbox.half_area() * (curr_num_primitives - 1);
            if (best_cost >= max_split_cost) {
                curr_node.num_primitives = curr_num_primitives;
                curr_node.first_primitive_index = begin;

                if (check_and_update_and_pop_stack()) continue;
                else break;
            }

            int left_node_index = num_nodes;
            Node &left_node = nodes[left_node_index];
            Node &right_node = nodes[left_node_index + 1];
            for (int i = begin; i < best_split_index; i++) {
                left_node.bbox.extend(bboxes[sorted_references[best_axis][i]]);
                marks[sorted_references[best_axis][i]] = true;
            }
            for (int i = best_split_index; i < end; i++) {
                right_node.bbox.extend(bboxes[sorted_references[best_axis][i]]);
                marks[sorted_references[best_axis][i]] = false;
            }

            int other_axis[2] = { (best_axis + 1) % 3, (best_axis + 2) % 3 };
            std::stable_partition(sorted_references[other_axis[0]].begin() + begin,
                                  sorted_references[other_axis[0]].begin() + end,
                                  [&](int i) { return marks[i]; });
            std::stable_partition(sorted_references[other_axis[1]].begin() + begin,
                                  sorted_references[other_axis[1]].begin() + end,
                                  [&](int i) { return marks[i]; });

            num_nodes += 2;
            curr_node.num_primitives = 0;
            curr_node.left_node_index = left_node_index;

            int left_size = best_split_index - begin;
            int right_size = end - best_split_index;

            if (left_size < right_size) {
                stack.push( { left_node_index + 1, best_split_index, end, depth + 1 } );
                node_index = left_node_index;
                begin = begin;
                end = best_split_index;
                depth = depth + 1;
            } else {
                stack.push( { left_node_index, begin, best_split_index, depth + 1 } );
                node_index = left_node_index + 1;
                begin = best_split_index;
                end = end;
                depth = depth + 1;
            }
        }

        // rearrange triangles based on sorted_references
        std::vector<Triangle> triangles_temp = *triangles_ptr;
        for (int i = 0; i < num_primitives; i++) {
            (*triangles_ptr)[i] = triangles_temp[sorted_references[0][i]];
        }
    }

    struct Intersection {
        int index;
        Triangle::Intersection triangle_intersection;
    };

    bool intersect_leaf(const Node &node, Ray &ray, Intersection &intersection) {
        bool hit_anything = false;
        for (int i = node.first_primitive_index;
             i < node.first_primitive_index + node.num_primitives;
             i++) {
            if ((*triangles_ptr)[i].intersect(ray, intersection.triangle_intersection)) {
                intersection.index = i;
                ray.tmax = intersection.triangle_intersection.t;
                hit_anything = true;
            }
        }
        return hit_anything;
    }

    bool traverse(Ray &ray, Intersection &intersection) {
        if (nodes[0].is_leaf())
            return intersect_leaf(nodes[0], ray, intersection);

        bool hit_anything = false;

        AABBIntersector aabb_intersector(ray);

        std::stack<int> stack;
        Node *left_node_ptr = &nodes[nodes[0].left_node_index];
        while (true) {
            Node *right_node_ptr = left_node_ptr + 1;

            float entry_left;
            if (aabb_intersector.intersect(left_node_ptr->bbox, entry_left)) {
                if (left_node_ptr->is_leaf()) {
                    hit_anything |= intersect_leaf(*left_node_ptr, ray, intersection);
                    left_node_ptr = nullptr;
                }
            } else {
                left_node_ptr = nullptr;
            }

            float entry_right;
            if (aabb_intersector.intersect(right_node_ptr->bbox, entry_right)) {
                if (right_node_ptr->is_leaf()) {
                    hit_anything |= intersect_leaf(*right_node_ptr, ray, intersection);
                    right_node_ptr = nullptr;
                }
            } else {
                right_node_ptr = nullptr;
            }

            if (left_node_ptr) {
                if (right_node_ptr) {
                    if (entry_left > entry_right)
                        std::swap(left_node_ptr, right_node_ptr);
                    stack.push(right_node_ptr->left_node_index);
                }
                left_node_ptr = &nodes[left_node_ptr->left_node_index];
            } else if (right_node_ptr) {
                left_node_ptr = &nodes[right_node_ptr->left_node_index];
            } else {
                if (stack.empty()) break;
                left_node_ptr = &nodes[stack.top()];
                stack.pop();
            }
        }

        return hit_anything;
    }

    static constexpr int MAX_DEPTH = 64;

    std::vector<Triangle> *triangles_ptr;
    int num_nodes;
    std::vector<Node> nodes;
};

#endif //BVH_BVH_HPP
