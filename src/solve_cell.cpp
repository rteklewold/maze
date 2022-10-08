#include <a_star.h>
#include <maze.h>

using namespace std;
using namespace ecn;

// a node is a x-y position, we move from 1 each time
class Position : public Point
{
    typedef std::unique_ptr<Position> PositionPtr;

public:
    // constructor from coordinates
    Position(int _x, int _y) : Point(_x, _y) {}

    // constructor from base ecn::Point
    Position(ecn::Point p) : Point(p.x, p.y) {}

    int distToParent()
    {
        // in cell-based motion, the distance to the parent is always 1
        return 1;
    }

    std::vector<PositionPtr> children()
    {
        // this method should return  all positions reachable from this one
        std::vector<PositionPtr> generated;

        const vector<pair<int,int>> neighboors{{-1,0},{1,0},{0,-1},{0,1}};

        for(const auto [dx,dy]: neighboors)
        {

           if(maze.isFree(x+dx, y+dy))
           {
             generated.emplace_back(new Position(x+dx, y+dy));
           }
        }

        return generated;
    }
};



int main( int argc, char **argv )
{
    // load file
    std::string filename = "/home/ecn/ecn_arpro/maze/mazes/maze3.png";
    //std::string filename = "/home/ecn/ecn_arpro/maze/mazes/maze.png";

    if(argc == 2)
        filename = std::string(argv[1]);

    // let Point know about this maze
    Position::maze.load(filename);

    // initial and goal positions as Position's
    Position start = Position::maze.start(),
             goal = Position::maze.end();

    // call A* algorithm
    ecn::Astar(start, goal);

    // save final image
    Position::maze.saveSolution("love");
    cv::waitKey(0);

}
