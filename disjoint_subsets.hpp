#ifndef _DISJOINT_SUBSETS_HPP_
#define _DISJOINT_SUBSETS_HPP_

#include <vector>

struct Node {
    int value;
    int height = 0;
    Node* parent;

    Node(int value, Node* parent) :
        value(value),
        parent(parent)
    {}
};

class DisjointSubsets {
    private:
        // Vector containing disjoint subsets as in-rooted trees
        std::vector<Node*> subsets;
        // Vector containing pointers to all Nodes in subsets
        // The position of each Node in the vector matches the Node's value
        std::vector<Node*> elements;

    public:
        // Class constructor that takes a vector of distinct integer elements
        // and populates the elements vector with appropriate Nodes as well
        // as the subsets vector with pointers to each Node in elements
        DisjointSubsets(const std::vector<int>& set) {
            subsets.reserve(set.size());
            elements.reserve(set.size());
            for (const int& i : set) {
                elements[i] = new Node(i, nullptr);
                subsets.push_back(elements[i]);
            }
        }

        // Class constructor that takes an integer, size, and populates the
        // elements vector with Nodes containing integer values from 0 to size
        // as well as the subsets vector with pointers to each Node in elements
        DisjointSubsets(int size) {
            subsets.reserve(size);
            elements.reserve(size);
            for (int i = 0; i < size; i++) {
                elements[i] = new Node(i, nullptr);
                subsets.push_back(elements[i]);
            }
        }

        ~DisjointSubsets() {
            for(auto& element : elements) {
                delete element;
            }
        }

        // Method that returns the root of the tree containing the Node with
        // the specified value x
        Node* Find(int x) const {
            Node* current = elements[x];

            while (current->parent != nullptr) {
                current = current->parent;
            }

            return current;
        }

        // Method that makes the root of the tree containing the value x
        // the parent of the root of the tree containing the value y
        // Returns false if x and y are in the same tree and true otherwise
        bool Union(int x, int y) {
            Node* x_root = Find(x);
            Node* y_root = Find(y);

            if (x_root->value == y_root->value) {
                return false;
            }
            else {
                y_root->parent = x_root;
                x_root->height++;

                subsets[y] = nullptr;
                return true;
            }
        }
};

#endif
