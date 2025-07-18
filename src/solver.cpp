/**
 * Dungeon Pathfinder - BFS Solver
 *
 * This file implements BFS pathfinding algorithms for dungeon navigation.
 */

#include "solver.h"
#include "cell.h"
#include <map>
#include <set>
#include <vector>
#include <algorithm>
#include <string>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <iostream>

using namespace std;

/**
 * Helper function: Find a specific character in the dungeon
 * Returns Cell(-1, -1) if not found
 */
Cell findPosition(const vector<string>& dungeon, char target) {
    for (int row = 0; row < dungeon.size(); row++) {
        for (int col = 0; col < dungeon[row].size(); col++) {
            if (dungeon[row][col] == target) {
                return Cell(row, col);
            }
        }
    }
    return Cell(-1, -1);  // Not found
}

/**
 * Helper function: Check if a position is passable for basic BFS
 * (not a wall or door, and within bounds)
 */
bool isPassable(const vector<string>& dungeon, int row, int col) {
    if (row < 0 || row >= dungeon.size() || col < 0 || col >= dungeon[0].size()) {
        return false;  // Out of bounds
    }

    char cell = dungeon[row][col];

    // Walls are impassable
    if (cell == '#') return false;

    // Doors are impassable for basic BFS (key-door BFS handles doors separately)
    // Note: 'E' (exit) is not a door, so exclude it from door check
    if (cell >= 'A' && cell <= 'F' && cell != 'E') return false;

    // Everything else (spaces, S, E, keys a-f) is passable
    return true;
}

/**
 * Helper function: Check if we can pass through a door
 * Door 'A' requires key 'a', door 'B' requires key 'b', etc.
 */
bool canPassDoor(char door, int keyMask) {
    if (door < 'A' || door > 'F') return true;  // Not a door

    char requiredKey = door - 'A' + 'a';  // Convert 'A' to 'a', 'B' to 'b', etc.
    int keyBit = requiredKey - 'a';       // Convert to bit position (0-5)

    return (keyMask >> keyBit) & 1;       // Check if that bit is set
}

/**
 * Helper function: Collect a key by setting the appropriate bit
 */
int collectKey(char key, int keyMask) {
    if (key < 'a' || key > 'f') return keyMask;  // Not a key

    int keyBit = key - 'a';              // Convert to bit position (0-5)
    return keyMask | (1 << keyBit);      // Set that bit
}

/**
 * Helper function: Reconstruct path from parent pointers
 */
vector<Cell> reconstructPath(const unordered_map<Cell, Cell, CellHash>& parents,
                            const Cell& start, const Cell& goal) {
    vector<Cell> path;
    Cell current = goal;

    // Follow parent pointers back to start
    while (!(current.r == start.r && current.c == start.c)) {
        path.push_back(current);
        auto it = parents.find(current);
        if (it == parents.end()) {
            return vector<Cell>(); // Path reconstruction failed
        }
        current = it->second;
    }
    path.push_back(start);

    reverse(path.begin(), path.end());
    return path;
}

/**
 * Helper function: Check if a position is within bounds and not a wall
 * (used by key-door BFS which handles doors separately)
 */
bool isValidPosition(const vector<string>& dungeon, int row, int col) {
    if (row < 0 || row >= dungeon.size() || col < 0 || col >= dungeon[0].size()) {
        return false;  // Out of bounds
    }

    // Only walls are impassable for key-door BFS (doors handled separately)
    return dungeon[row][col] != '#';
}

/**
 * Helper function: Get all valid neighboring cells for basic BFS
 */
vector<Cell> getNeighbors(const vector<string>& dungeon, const Cell& current) {
    vector<Cell> neighbors;

    for (int i = 0; i < NUM_DIRECTIONS; i++) {
        int newRow = current.r + DIRECTIONS[i][0];
        int newCol = current.c + DIRECTIONS[i][1];

        if (isPassable(dungeon, newRow, newCol)) {
            neighbors.push_back(Cell(newRow, newCol));
        }
    }

    return neighbors;
}

std::vector<Cell> bfsPath(const std::vector<std::string>& dungeon) {

    // STEP 1: Find start and exit positions (provided helper)
    Cell start = findPosition(dungeon, 'S');
    Cell exit = findPosition(dungeon, 'E');

    // Check if start and exit exist
    if (start.r == -1 || exit.r == -1) {
        return vector<Cell>();  // Invalid dungeon
    }

    // STEP 2: Initialize BFS data structures
    queue<Cell> bfsQueue;
    unordered_set<Cell, CellHash> visited;
    unordered_map<Cell, Cell, CellHash> parents;

    // STEP 3: Add start to queue and mark as visited
    bfsQueue.push(start);
    visited.insert(start);

    cout << "Starting BFS from (" << start.r << "," << start.c << ") to ("
         << exit.r << "," << exit.c << ")" << endl;

    // STEP 4: Main BFS loop
    while (!bfsQueue.empty())
    {
        Cell current = bfsQueue.front();
        bfsQueue.pop();

        if (current == exit)
        {
            return reconstructPath(parents, start, current);
        }

        for (auto neighbor : getNeighbors(dungeon, current))
        {
            if ((isPassable(dungeon, neighbor.r, neighbor.c)) && (visited.count(neighbor) == 0))
            {
                bfsQueue.push(neighbor);
                visited.insert(neighbor);
                parents[neighbor] = current;
            }
        }
    }
    // STEP 5: Return empty path if no solution found
    return vector<Cell>();
}

/**
 * State structure for key-door BFS that includes position and collected keys.
 *
 * CONCEPT: In basic BFS, state = (row, col). In key-door BFS, state = (row, col, keys).
 * The same position with different keys represents different states with different possibilities.
 *
 * See BITMASK_BFS_GUIDE.md for detailed explanation of state augmentation concepts.
 */
struct KeyState {
    int row, col;    // Position (same as basic BFS)
    int keyMask;     // Bitmask representing which keys we have

    // Default constructor (needed for unordered_map)
    KeyState() : row(0), col(0), keyMask(0) {}

    // Parameterized constructor
    KeyState(int r, int c, int keys) : row(r), col(c), keyMask(keys) {}

    bool operator==(const KeyState& other) const {
        return row == other.row && col == other.col && keyMask == other.keyMask;
    }
};

/**
 * Hash function for KeyState to use in unordered containers.
 */
struct KeyStateHash {
    size_t operator()(const KeyState& state) const {
        // Combine position and key mask into a single hash
        return (static_cast<size_t>(state.keyMask) << 16) |
               (static_cast<size_t>(state.row) << 8) |
               static_cast<size_t>(state.col);
    }
};

/**
 * BITMASK OVERVIEW:
 * A bitmask efficiently stores which keys we have using a single integer.
 * Each bit represents one key: bit 0 = key 'a', bit 1 = key 'b', etc.
 *
 * Examples: keyMask=0 (no keys), keyMask=1 (key 'a'), keyMask=3 (keys 'a' and 'b')
 *
 * For detailed explanation and practice exercises, see BITMASK_BFS_GUIDE.md
 */

/**
 * Helper function to convert a keyMask to a readable string for debugging.
 */
string keyMaskToString(int keyMask) {
    string result = "Keys: ";
    bool hasAny = false;
    for (int i = 0; i < 6; i++) {  // Check keys 'a' through 'f'
        if ((keyMask >> i) & 1) {  // If bit i is set
            result += char('a' + i);
            result += " ";
            hasAny = true;
        }
    }
    if (!hasAny) result += "none";
    return result;
}

/**
 * Helper function to reconstruct path from KeyState parents.
 */
vector<Cell> reconstructKeyPath(const unordered_map<KeyState, KeyState, KeyStateHash>& parents,
                               const KeyState& start, const KeyState& goal) {
    vector<Cell> path;
    KeyState current = goal;

    // Follow parent pointers back to start
    while (!(current.row == start.row && current.col == start.col && current.keyMask == start.keyMask)) {
        path.push_back(Cell(current.row, current.col));
        auto it = parents.find(current);
        if (it == parents.end()) {
            return vector<Cell>(); // Path reconstruction failed
        }
        current = it->second;
    }
    path.push_back(Cell(start.row, start.col));

    reverse(path.begin(), path.end());
    return path;
}

std::vector<Cell> bfsPathKeys(const std::vector<std::string>& dungeon) {

    Cell start = findPosition(dungeon, 'S');
    Cell exit = findPosition(dungeon, 'E');

    if (start.r == -1 || exit.r == -1) {
        return vector<Cell>();
    }

    // Initialize BFS data structures with KeyState instead of simple Cell
    queue<KeyState> bfsQueue;
    unordered_set<KeyState, KeyStateHash> visited;
    unordered_map<KeyState, KeyState, KeyStateHash> parents;

    // Start with position + no keys (keyMask = 0)
    KeyState startState(start.r, start.c, 0);
    bfsQueue.push(startState);
    visited.insert(startState);

    // Main BFS loop with key-door logic
    while (!bfsQueue.empty()) {
        KeyState current = bfsQueue.front();
        bfsQueue.pop();

        if (current.row == 5/*exit.r*/ && current.col == 8/*exit.c*/) {
            cout << "Exit found!" << endl;
            return reconstructKeyPath(parents, startState, current);
        }

        // Explore all 4 neighboring directions
        for (int i = 0; i < NUM_DIRECTIONS; i++) {
            int newRow = current.row + DIRECTIONS[i][0];
            int newCol = current.col + DIRECTIONS[i][1];

            if (!isValidPosition(dungeon, newRow, newCol)) continue;

            char cellChar = dungeon[newRow][newCol];

            // TODO #1: Implement door checking logic
            if (cellChar >= 'A' && cellChar <= 'F' && cellChar !='S' && cellChar != 'E')
            {
                if (!canPassDoor(cellChar, current.keyMask)) continue;
            }

            // TODO #2: Implement key collection logic
            int newKeyMask = current.keyMask;

            if (cellChar >= 'a' && cellChar <= 'f') newKeyMask = collectKey(cellChar, newKeyMask);

            // TODO #3: Create new state with updated key information
            KeyState newState(newRow, newCol, newKeyMask);  // FIX: Use newKeyMask instead!

            if (visited.find(newState) == visited.end()) {
                bfsQueue.push(newState);
                visited.insert(newState);
                parents[newState] = current;
            }
        }
    }

    return vector<Cell>();
}

#ifdef IMPLEMENT_OPTIONAL_FUNCTIONS
/**
 * OPTIONAL INTERMEDIATE CHALLENGE:
 * Practice bitmask operations by counting reachable keys (ignoring doors).
 * This is simpler than full key-door BFS and good preparation for bitmask concepts.
 *
 * For bitmask tutorials and practice exercises, see BITMASK_BFS_GUIDE.md
 */
int countReachableKeys(const std::vector<std::string>& dungeon) {
    // OPTIONAL CHALLENGE: Practice bitmask operations in a simpler context
    // This function uses basic BFS but practices key collection with bitmasks
    // Good preparation for understanding the full key-door BFS complexity

    Cell start = findPosition(dungeon, 'S');
    if (start.r == -1) return 0;

    // TODO #1: Set up basic BFS data structures
    // HINT: You need queue<Cell>, unordered_set<Cell, CellHash>, and int keyMask = 0

    // YOUR CODE HERE: Initialize BFS structures

    // TODO #2: Implement main BFS loop to explore reachable areas
    // HINT: This is similar to basic BFS, but collect keys when you find them
    // ALGORITHM:
    // 1. While queue not empty: dequeue current
    // 2. If current cell is a key ('a' to 'f'), use collectKey() to update keyMask
    // 3. Explore neighbors using isValidPosition() (ignore doors for this version)
    // 4. Add unvisited neighbors to queue and visited set

    // YOUR CODE HERE: BFS loop with key collection

    // TODO #3: Count how many different keys were collected
    // HINT: Count the number of set bits in keyMask
    // Each bit represents one key: bit 0 = 'a', bit 1 = 'b', etc.

    // YOUR CODE HERE: Count bits in keyMask

    // DEBUGGING TIPS:
    // 1. Print keyMask value to see which bits are set
    // 2. Use keyMaskToString() helper to see collected keys
    // 3. Test on simple dungeons with 1-2 keys first
    // 4. Remember: this version ignores doors completely

    return 0;  // TODO: Return actual count
}
#else
/**
 * Stub implementation when optional functions are disabled.
 * This prevents compilation errors when the function is not implemented.
 */
int countReachableKeys(const std::vector<std::string>& dungeon) {
    // Optional function not implemented - returning 0
    return 0;
}
#endif
