#include <iostream>
#include <cstdlib>  // atoi
#include <stdlib.h>
#include "Maze.h"
#include "MazeDefinitions.h"
#include "PathFinder.h"
#include <sstream>
#include <stack>

/**
 * Demo of a PathFinder implementation.
 *
 * Do not use a left/right wall following algorithm, as most
 * Micromouse mazes are designed for such algorithms to fail.
 */
class LeftWallFollower : public PathFinder {
public:
    LeftWallFollower(bool shouldPause = false) : pause(shouldPause) {
        shouldGoForward = false;
        visitedStart = false;
    }
    
    MouseMovement nextMovement(unsigned x, unsigned y, const Maze &maze) {
        const bool frontWall = maze.wallInFront();
        const bool leftWall = maze.wallOnLeft();
        
        // Pause at each cell if the user requests it.
        // It allows for better viewing on command line.
        if (pause) {
            std::cout << "Hit enter to continue..." << std::endl;
            std::cin.ignore(10000, '\n');
            std::cin.clear();
        }
        
        std::cout << maze.draw(5) << std::endl << std::endl;
        
        // If we somehow miraculously hit the center
        // of the maze, just terminate and celebrate!
        if (isAtCenter(x, y)) {
            std::cout << "Found center! Good enough for the demo, won't try to get back." << std::endl;
            return Finish;
        }
        
        // If we hit the start of the maze a second time, then
        // we couldn't find the center and never will...
        if (x == 0 && y == 0) {
            if (visitedStart) {
                std::cout << "Unable to find center, giving up." << std::endl;
                return Finish;
            }
            else {
                visitedStart = true;
            }
        }
        
        // If we have just turned left, we should take that path!
        if (!frontWall && shouldGoForward) {
            shouldGoForward = false;
            return MoveForward;
        }
        
        // As long as nothing is in front and we have
        // a wall to our left, keep going forward!
        if (!frontWall && leftWall) {
            shouldGoForward = false;
            return MoveForward;
        }
        
        // If our forward and left paths are blocked
        // we should try going to the right!
        if (frontWall && leftWall) {
            shouldGoForward = false;
            return TurnClockwise;
        }
        
        // Lastly, if there is no left wall we should take that path!
        if (!leftWall) {
            shouldGoForward = true;
            return TurnCounterClockwise;
        }
        
        // If we get stuck somehow, just terminate.
        std::cout << "Got stuck..." << std::endl;
        return Finish;
    }
    
protected:
    // Helps us determine that we should go forward if we have just turned left.
    bool shouldGoForward;
    
    // Helps us determine if we've made a loop around the maze without finding the center.
    bool visitedStart;
    
    // Indicates we should pause before moving to next cell.
    // Useful for command line usage.
    const bool pause;
    
    bool isAtCenter(unsigned x, unsigned y) const {
        unsigned midpoint = MazeDefinitions::MAZE_LEN / 2;
        
        if (MazeDefinitions::MAZE_LEN % 2 != 0) {
            return x == midpoint && y == midpoint;
        }
        
        return  (x == midpoint     && y == midpoint) ||
        (x == midpoint - 1 && y == midpoint) ||
        (x == midpoint     && y == midpoint - 1) ||
        (x == midpoint - 1 && y == midpoint - 1);
    }
};











struct Coord
{
public:
    Coord() {
        m_isProcessed = false;
        m_distance = 0;
    }
    void process()
    {
        m_isProcessed = true;
    }
    bool isProcessed()
    {
        return m_isProcessed;
    }
    int m_distance;
    int m_xPos;
    int m_yPos;
private:
    bool m_isProcessed;
    
};

class Micromouse : public PathFinder {
public:
    Micromouse(bool shouldPause = false) : pause(shouldPause) {
        shouldGoForward = false;
        visitedStart = false;
        heading = NORTH;
        calculateManahattan(7, 7);
    }
    
    void calculateManahattan(int goalX, int goalY)
    {
        //set Manhattan Distances
        int xMid = goalX;
        int yMid = goalY;
        
        //Quadrant 1
        for (int i = 0; i < 8; i++) {
            for (int j = 0; j < 8; j++) {
                c_array[i][j].m_distance = abs(xMid - i) + abs(yMid - j);
            }
        }
        //Quadrant 2
        for (int i = 8; i < 16; i++) {
            for (int j = 0; j < 8; j++) {
                c_array[i][j].m_distance = abs(xMid - i) - 1 + abs(yMid - j);
            }
        }
        //Quadrant 3
        for (int i = 8; i < 16; i++) {
            for (int j = 8; j < 16; j++) {
                c_array[i][j].m_distance = abs(xMid - i) + abs(yMid - j) - 2;
            }
        }
        
        //Quadrant 4
        for (int i = 0; i < 8; i++) {
            for (int j = 8; j < 16; j++) {
                c_array[i][j].m_distance = abs(xMid - i) + abs(yMid - j) - 1;
            }
        }
        
        for (int i = 0; i < 16; i++) {
            for (int j = 0; j < 16; j++) {
                std::cout << c_array[i][j].m_distance;
                std::cout << " ";
            }
            std::cout << "" << std::endl;
        }
        for (int i = 0; i < 16; i++)
        {
            for (int j = 0; j < 16; j++)
            {
                c_array[i][j].m_xPos = i;
                c_array[i][j].m_yPos = j;
            }
        }
    }
    
    MouseMovement nextMovement(unsigned x, unsigned y, const Maze &maze) {
        const bool frontWall = maze.wallInFront();
        const bool leftWall = maze.wallOnLeft();
        const bool rightWall = maze.wallOnRight();
        
        
        
        // Pause at each cell if the user requests it.
        // It allows for better viewing on command line.
        if (pause) {
            std::cout << "Hit enter to continue..." << std::endl;
            std::cin.ignore(10000, '\n');
            std::cin.clear();
        }
        
        std::cout << maze.draw(5) << std::endl << std::endl;

        if (isAtCenter(x, y))
        {
            
        }
        
        if (x == 0 && y == 0) {
            if (visitedStart) {
//                std::cout << "Time to go back to the center" << std::endl;
//                calculateManahattan(0, 0);
//                floodFill(x, y);
//                heading = opposite(heading);
//                return TurnAround;
            }
            else {
                visitedStart = true;
            }
        }
        
        
        //Set wall positions
        //LEFTWALL
        if (leftWall) {
            if (heading == NORTH) {
                if (x != 0){
                    mazeVertWalls.set(x - 1, y);
                }
            }
            else if (heading == SOUTH) {
                mazeVertWalls.set(x, y);
                
            }
            else if (heading == WEST) {
                if (y != 0){
                    mazeHorWalls.set(x, y - 1);
                }
                
            }
            else if (heading == EAST) {
                mazeHorWalls.set(x, y);
            }
        }
        
        //RIGHTWALL
        if (rightWall) {
            if (heading == NORTH) {
                mazeVertWalls.set(x, y);
                
            }
            else if (heading == SOUTH) {
                if (x != 0){
                    mazeVertWalls.set(x - 1, y);
                }
                
            }
            else if (heading == WEST) {
                mazeHorWalls.set(x, y);
                
            }
            else if (heading == EAST) {
                if (y != 0){
                    mazeHorWalls.set(x, y - 1);
                }
                
            }
            
        }
        //FRONTWALL
        if (frontWall)
        {
            if (heading == NORTH)
            {
                mazeHorWalls.set(x, y);
            }
            else if (heading == SOUTH)
            {
                if (y != 0)
                {
                    mazeHorWalls.set(x, y - 1);
                }
            }
            else if (heading == WEST)
            {
                if (x != 0)
                {
                    mazeVertWalls.set(x - 1, y);
                }
                
            }
            else if (heading == EAST)
            {
                mazeVertWalls.set(x, y);
            }
        }
        //print out bitvector
//        std::cout << "mazeHorWalls" << std::endl;
//        for (int i = 15; i >= 0; i--)
//        {
//            for (int j = 0; j < 16; j++)
//            {
//                std::cout << "" << mazeHorWalls.get(j, i);
//            }
//            std::cout << std::endl;
//        }
//        std::cout << std::endl;
//        
//        std::cout << "mazeVertWalls" << std::endl;
//        for (int i = 15; i >= 0; i--)
//        {
//            for (int j = 0; j < 16; j++)
//            {
//                std::cout << "" << mazeVertWalls.get(j, i);
//            }
//            std::cout << std::endl;
//        }
//        std::cout << "mazeHorWalls" << std::endl;
//        for (int i = 0; i < 16; i++)
//        {
//            for (int j = 0; j < 16; j++)
//            {
//                std::cout << "" << mazeHorWalls.get(i, j);
//            }
//            std::cout << std::endl;
//        }
//        std::cout << std::endl;
//        
//        std::cout << "mazeVertWalls" << std::endl;
//        for (int i = 0; i < 16; i++)
//        {
//            for (int j = 0; j < 16; j++)
//            {
//                std::cout << "" << mazeVertWalls.get(i, j);
//            }
//            std::cout << std::endl;
//        }

        std::cout << std::endl;
        int frontX = 0;
        int frontY = 0;
        int rightX = 0;
        int rightY = 0;
        int leftX = 0;
        int leftY = 0;
        
        switch (heading)
        {
            case NORTH:
                frontX = x;
                frontY = y + 1;
                rightX = x + 1;
                rightY = y;
                leftX = x - 1;
                leftY = y;
                break;
            case SOUTH:
                frontX = x;
                frontY = y - 1;
                rightX = x - 1;
                rightY = y;
                leftX = x + 1;
                leftY = y;
                break;
            case EAST:
                frontX = x + 1;
                frontY = y;
                rightX = x;
                rightY = y - 1;
                leftX = x;
                leftY = y + 1;
                break;
            case WEST:
                frontX = x - 1;
                frontY = y;
                rightX = x;
                rightY = y + 1;
                leftX = x;
                leftY = y - 1;
                break;
            default:
                break;
        }
//        std::cout << "front = (" << frontX << ", " << frontY << ")" << std::endl;
//        std::cout << "left = (" << leftX << ", " << leftY << ")" << std::endl;
//        std::cout << "right = (" << rightX << ", " << rightY << ")" << std::endl;
        
        
        //If there is no front wall, and Manhattan Distance of front cell is less than current cell, move forward
        if (!frontWall && c_array[x][y].m_distance > c_array[frontX][frontY].m_distance)
        {
            return MoveForward;
        }
        
        //If there is no right wall, and Manhattan Distance of right cell is less than current cell, turn right
        if (!rightWall && c_array[x][y].m_distance > c_array[rightX][rightY].m_distance)
        {
            heading = clockwise(heading);
            return TurnClockwise;
        }
        
        //If there is no left wall, and Manhattan Distance of left cell is less than current cell, turn left
        if (!leftWall && c_array[x][y].m_distance > c_array[leftX][leftY].m_distance)
        {
            heading = counterClockwise(heading);
            return TurnCounterClockwise;
        }
        
        //If there are walls on all three sides, turn clockwise and try again
        if (frontWall && rightWall && leftWall)
        {
            heading = clockwise(heading);
            return TurnClockwise;
        }
        
        //If the Manhattan Distance of the cell behind you is less than current cell, and there is no wall between current cell and there, turn around
        if ((heading == NORTH && c_array[x][y].m_distance > c_array[x][y-1].m_distance && !isWallBetween(c_array[x][y], c_array[x][y-1])) || (heading == SOUTH && c_array[x][y].m_distance > c_array[x][y+1].m_distance && !isWallBetween(c_array[x][y], c_array[x][y+1])) || (heading == EAST && c_array[x][y].m_distance > c_array[x-1][y].m_distance && !isWallBetween(c_array[x][y], c_array[x-1][y])) || (heading == WEST && c_array[x][y].m_distance > c_array[x+1][y].m_distance && !isWallBetween(c_array[x][y], c_array[x+1][y])))
        {
            heading = opposite(heading);
            return TurnAround;
        }
        
        //If none of these cases are satisfied
        //If still not at destination, run floodfill and rerun nextMovement
        if (c_array[x][y].m_distance != 0)
        {
            std::cout << "Running Floodfill" << std::endl;
            floodFill(x, y);
            return nextMovement(x, y, maze);
        }
        //If we are at destination, set other destination and run floodfill on all (visited?) cells
        else
        {
            shouldGoToCenter = !shouldGoToCenter;
            if (shouldGoToCenter)
            {
                std::cout << "Went back to the beginning. Now to go back to the center" << std::endl;
                calculateManahattan(7, 7);
                
                //Call Floodfill on every cell
                for (int i = 0; i < 16; i++)
                {
                    for (int j = 0; j < 16; j++)
                    {//Might only want to run floodfill if cell has been visited. Could lead to infinite loop otherwise. Will see.
                        floodFill(i, j);
                    }
                }
                heading = opposite(heading);
                return TurnAround;
            }
            else
            {
                std::cout << "Found center! Now to go back to the beginning" << std::endl;
                calculateManahattan(0, 0);
                floodFill(x, y);
                heading = opposite(heading);
                return TurnAround;
            }
        }
    }
    
protected:
    // Helps us determine that we should go forward if we have just turned left.
    bool shouldGoForward;
    bool shouldGoToCenter = true;
    
    BitVector256 mazeHorWalls;
    BitVector256 mazeVertWalls;
    
    const int MAZE_W = 16;
    const int MAZE_L = 16;
    
    Coord c_array[16][16];
    
    Dir heading;
    
    // Helps us determine if we've made a loop around the maze without finding the center.
    bool visitedStart;
    
    // Indicates we should pause before moving to next cell.
    // Useful for command line usage.
    const bool pause;
    
    bool isAtCenter(unsigned x, unsigned y) const {
        unsigned midpoint = MazeDefinitions::MAZE_LEN / 2;
        
        if (MazeDefinitions::MAZE_LEN % 2 != 0) {
            return x == midpoint && y == midpoint;
        }
        
        return  (x == midpoint     && y == midpoint) ||
        (x == midpoint - 1 && y == midpoint) ||
        (x == midpoint     && y == midpoint - 1) ||
        (x == midpoint - 1 && y == midpoint - 1);
    }
    
    bool isWallBetween(Coord pos1, Coord pos2)
    {
        if (pos1.m_xPos == pos2.m_xPos)
        {
            if (pos1.m_yPos > pos2.m_yPos)
            {
                return mazeHorWalls.get(pos2.m_xPos, pos2.m_yPos);
            }
            else if (pos1.m_yPos < pos2.m_yPos)
            {
                return mazeHorWalls.get(pos1.m_xPos, pos1.m_yPos);
            }
            else return false;
        }
        else if (pos2.m_yPos == pos1.m_yPos)
        {
            if (pos1.m_xPos > pos2.m_xPos)
            {
                return mazeVertWalls.get(pos2.m_xPos, pos2.m_yPos);
            }
            else if (pos1.m_xPos < pos2.m_xPos)
            {
                return mazeVertWalls.get(pos1.m_xPos, pos1.m_yPos);
            }
            else return false;
        }
        return false;
    }
    
    void floodFill(int xPos, int yPos)
    {
        std::stack<Coord> stack;
        stack.push(c_array[xPos][yPos]); //push current cell onto stack
        while (!stack.empty()) //while the stack is not empty
        {
            Coord cur = stack.top();
            stack.pop();
            cur.process();
            if (cur.m_distance == 0)
            {
                continue;
            }
            int shortest = 500; //should be some large number that every cell's Manhattan Distance WILL be less than
            for (int i = 0; i < 4; i++)
            {
                Coord neighbor;
                //Set neighbor to the currect coordinate
                switch (i)
                {
                    case NORTH:
                        if (cur.m_yPos >= 15)
                            continue;
                        neighbor = c_array[cur.m_xPos][cur.m_yPos + 1];
                        break;
                    case SOUTH:
                        if (cur.m_yPos <= 0)
                            continue;
                        neighbor = c_array[cur.m_xPos][cur.m_yPos - 1];
                        break;
                    case EAST:
                        if (cur.m_xPos >= 15)
                            continue;
                        neighbor = c_array[cur.m_xPos + 1][cur.m_yPos];
                        break;
                    case WEST:
                        if (cur.m_xPos <= 0)
                            continue;
                        neighbor = c_array[cur.m_xPos - 1][cur.m_yPos];
                        break;
                }
                //Check if there is a wall between cur and neighbor
                if (!isWallBetween(cur, neighbor))
                {
                    //If neighbor has a shorter ManHattan Distance than current shortest, update current shortest
                    if (neighbor.m_distance < shortest)
                    {
                        shortest = neighbor.m_distance;
                    }
                    //Mark the neighbor as processed
                    if (!neighbor.isProcessed())
                    {
                        neighbor.process();
                    }
                }
            }
            //If none of the neighbors have a shorter path, something went wrong. Move on before it becomes a problem
            if (shortest == 500)
            {
                continue;
            }
            //If the current cell's Manhattan Distance is shortest+1, this is correct. Nothing needs to be updated
            if (cur.m_distance == (shortest + 1))
            {
                continue;
            }
            //If it is not, set the current cell's Manhattan Distance to shortest+1. This is where Manhattan Distances actually get updated for any given cell
            else
            {
                c_array[cur.m_xPos][cur.m_yPos].m_distance = shortest + 1;
            }
            for (int i = 0; i < 4; i++)
            {
                Coord neighbor;
                //Set neighbor to the currect coordinate
                switch (i)
                {
                    case NORTH:
                        if (cur.m_yPos >= 15)
                            continue;
                        neighbor = c_array[cur.m_xPos][cur.m_yPos + 1];
                        break;
                    case SOUTH:
                        if (cur.m_yPos <= 0)
                            continue;
                        neighbor = c_array[cur.m_xPos][cur.m_yPos - 1];
                        break;
                    case EAST:
                        if (cur.m_xPos >= 15)
                            continue;
                        neighbor = c_array[cur.m_xPos + 1][cur.m_yPos];
                        break;
                    case WEST:
                        if (cur.m_xPos <= 0)
                            continue;
                        neighbor = c_array[cur.m_xPos - 1][cur.m_yPos];
                        break;
                }
                //Check if there is a wall between cur and neighbor
                if (!isWallBetween(cur, neighbor))
                {
                    //Now update distances on all non-walled neighbors
                    stack.push(neighbor);
                }
            }
        }
    }
    
    
    virtual std::string getInfo(unsigned x, unsigned y, size_t maxInfoLen)
    {
        //Display Manhattan Distances
        std::stringstream convert;
        convert << c_array[x][y].m_distance;
        return convert.str();
        //Display coordinates of each cell
//        std::stringstream coordinate;
//        coordinate << "(" << c_array[x][y].m_xPos << "," << c_array[x][y].m_yPos << ")";
//        return coordinate.str();
        //Display if has a wall with upper neighbor
//        Coord pos1 = c_array[x][y];
//        Coord pos2 = c_array[x][y+1];
//        bool res = isWallBetween(c_array[x][y], c_array[x][y+1]);
//        if (res)
//            return "Yes";
//        else
//            return "No";
//        return "";
    }
    
};


int main(int argc, char * argv[]) {
    MazeDefinitions::MazeEncodingName mazeName = MazeDefinitions::MAZE_CAMM_2012;
    bool pause = true;
    
    // Since Windows does not support getopt directly, we will
    // have to parse the command line arguments ourselves.
    
    // Skip the program name, start with argument index 1
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-m") == 0 && i + 1 < argc) {
            int mazeOption = atoi(argv[++i]);
            if (mazeOption < MazeDefinitions::MAZE_NAME_MAX && mazeOption > 0) {
                mazeName = (MazeDefinitions::MazeEncodingName)mazeOption;
            }
        }
        else if (strcmp(argv[i], "-p") == 0) {
            pause = true;
        }
        else {
            std::cout << "Usage: " << argv[0] << " [-m N] [-p]" << std::endl;
            std::cout << "\t-m N will load the maze corresponding to N, or 0 if invalid N or missing option" << std::endl;
            std::cout << "\t-p will wait for a newline in between cell traversals" << std::endl;
            return -1;
        }
    }
    
    Micromouse leftWallFollower(pause);
    Maze maze(mazeName, &leftWallFollower);
    std::cout << maze.draw(5) << std::endl << std::endl;
    
    maze.start();
}