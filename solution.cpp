#include <iostream>
#include <math.h>
#include <vector>
#include <iterator>
#include <fstream>
#include "src/matplotlibcpp.h" //Graph Library

using namespace std;
namespace plt = matplotlibcpp;

// Map class
class Map {
public:
    const static int mapHeight = 300;
    const static int mapWidth = 150;
    vector<vector<double> > map = GetMap();
    vector<vector<int> > grid = MaptoGrid();
    vector<vector<int> > heuristic = GenerateHeuristic();

private:
    // Read the file and get the map
    vector<vector<double> > GetMap()
    {
        vector<vector<double> > mymap(mapHeight, vector<double>(mapWidth));
        ifstream myReadFile;
        myReadFile.open("map.txt");

        while (!myReadFile.eof()) {
            for (int i = 0; i < mapHeight; i++) {
                for (int j = 0; j < mapWidth; j++) {
                    myReadFile >> mymap[i][j];
                }
            }
        }
        return mymap;
    }

    //Convert the map to 1's and 0's
    vector<vector<int> > MaptoGrid()
    {
        vector<vector<int> > grid(mapHeight, vector<int>(mapWidth));
        for (int x = 0; x < mapHeight; x++) {
            for (int y = 0; y < mapWidth; y++) {
                if (map[x][y] == 0) //unkown state
                    grid[x][y] = 1;

                else if (map[x][y] > 0) //Occupied state
                    grid[x][y] = 1;

                else //Free state
                    grid[x][y] = 0;
            }
        }

        return grid;
    }

    // Generate a Manhattan Heuristic Vector
    vector<vector<int> > GenerateHeuristic()
    {
        vector<vector<int> > heuristic(mapHeight, vector<int>(mapWidth));
        int goal[2] = { 60, 50 };
        for (int i = 0; i < heuristic.size(); i++) {
            for (int j = 0; j < heuristic[0].size(); j++) {
                int xd = goal[0] - i;
                int yd = goal[1] - j;
                // Manhattan Distance
                   int d = abs(xd) + abs(yd);
                // Euclidian Distance
                // double d = sqrt(xd * xd + yd * yd);
                // Chebyshev distance
                // int d = max(abs(xd), abs(yd));
                heuristic[i][j] = d;
            }
        }
        return heuristic;
    }
};

// Planner class
class Planner : Map {
public:
    int start[2] = { 230, 145 };
    int goal[2] = { 60, 50 };
    int cost = 1;

    string movements_arrows[4] = { "^", "<", "v", ">" };

    vector<vector<int> > movements{
        { -1, 0 },
        { 0, -1 },
        { 1, 0 },
        { 0, 1 }
    };

    vector<vector<int> > path;
};

// Printing vectors of any type
template <typename T>
void print2DVector(T Vec)
{
    for (int i = 0; i < Vec.size(); ++i) {
        for (int j = 0; j < Vec[0].size(); ++j) {
            cout << Vec[i][j] << ' ';
        }
        cout << endl;
    }
}

Planner search(Map map, Planner planner)
{
    // Create a closed 2 array filled with 0s and first element 1
    vector<vector<int> > closed(map.mapHeight, vector<int>(map.mapWidth));
    closed[planner.start[0]][planner.start[1]] = 1;

    // Create expand array filled with -1
    vector<vector<int> > expand(map.mapHeight, vector<int>(map.mapWidth, -1));

    // Create action array filled with -1
    vector<vector<int> > action(map.mapHeight, vector<int>(map.mapWidth, -1));

    // Defined the quadruplet values
    int x = planner.start[0];
    int y = planner.start[1];
    int g = 0;
    int f = g + map.heuristic[x][y];

    // Store the expansions
    vector<vector<int> > open;
    open.push_back({ f, g, x, y });

    // Flags and Counts
    bool found = false;
    bool resign = false;
    int count = 0;

    int x2;
    int y2;

    // While I am still searching for the goal and the problem is solvable
    while (!found && !resign) {
        // Resign if no values in the open list and you can't expand anymore
        if (open.size() == 0) {
            resign = true;
            cout << "Failed to reach a goal" << endl;
        }
        // Keep expanding
        else {
            // Remove quadruplets from the open list
            sort(open.begin(), open.end());
            reverse(open.begin(), open.end());
            vector<int> next;
            // Stored the poped value into next
            next = open.back();
            open.pop_back();

            x = next[2];
            y = next[3];
            g = next[1];

            // Fill the expand vectors with count
            expand[x][y] = count;
            count += 1;

            // Check if we reached the goal:
            if (x == planner.goal[0] && y == planner.goal[1]) {
                found = true;
                //cout << "[" << g << ", " << x << ", " << y << "]" << endl;
            }

            //else expand new elements
            else {
                for (int i = 0; i < planner.movements.size(); i++) {
                    x2 = x + planner.movements[i][0];
                    y2 = y + planner.movements[i][1];
                    if (x2 >= 0 && x2 < map.grid.size() && y2 >= 0 && y2 < map.grid[0].size()) {
                        if (closed[x2][y2] == 0 and map.grid[x2][y2] == 0) {
                            int g2 = g + planner.cost;
                            f = g2 + map.heuristic[x2][y2];
                            open.push_back({ f, g2, x2, y2 });
                            closed[x2][y2] = 1;
                            action[x2][y2] = i;
                        }
                    }
                }
            }
        }
    }

    // Print the expansion List
    print2DVector(expand);

    // Find the path with robot orientation
    vector<vector<string> > policy(map.mapHeight, vector<string>(map.mapWidth, "-"));

    // Going backward
    x = planner.goal[0];
    y = planner.goal[1];
    policy[x][y] = '*';

    while (x != planner.start[0] or y != planner.start[1]) {
        x2 = x - planner.movements[action[x][y]][0];
        y2 = y - planner.movements[action[x][y]][1];
        // Store the  Path in a vector
        planner.path.push_back({ x2, y2 });
        policy[x2][y2] = planner.movements_arrows[action[x][y]];
        x = x2;
        y = y2;
    }

    // Print the robot path
    cout << endl;
    print2DVector(policy);

    return planner;
}

void visualization(Map map, Planner planner)
{
    //Graph Format
    plt::title("Path");
    plt::xlim(0, map.mapHeight);
    plt::ylim(0, map.mapWidth);

    // Draw every grid of the map:
    for (double x = 0; x < map.mapHeight; x++) {
        cout << "Remaining Rows= " << map.mapHeight - x << endl;
        for (double y = 0; y < map.mapWidth; y++) {
            if (map.map[x][y] == 0) { //Green unkown state
                plt::plot({ x }, { y }, "g.");
            }
            else if (map.map[x][y] > 0) { //Black occupied state
                plt::plot({ x }, { y }, "k.");
            }
            else { //Red free state
                plt::plot({ x }, { y }, "r.");
            }
        }
    }

    // Plot start and end states
    plt::plot({ (double)planner.start[0] }, { (double)planner.start[1] }, "bo");
    plt::plot({ (double)planner.goal[0] }, { (double)planner.goal[1] }, "b*");

    // Plot the robot path
    for (int i = 0; i < planner.path.size(); i++) {
        plt::plot({ (double)planner.path[i][0] }, { (double)planner.path[i][1] }, "b.");
    }

    //Save the image and close the plot
    plt::save("./Images/Path.png");
    plt::clf();
}

int main()
{
    // Instantiate a planner and map objects
    Map map;
    Planner planner;
    // Generate the shortest Path using the Astar algorithm
    planner = search(map, planner);
    // Plot the Map and the path generated
    visualization(map, planner);

    return 0;
}

