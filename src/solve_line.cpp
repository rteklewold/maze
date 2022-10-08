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

    //new Constructor
    Position(int _x,int _y, double d): Point(_x,_y){
        this->distance=d;
    }

    int distToParent()
    {
        // in cell-based motion, the distance to the parent is always 1
        return this->distance;
    }

    bool is_corridor(int ax, int ay){
        bool is_corr=false;
        bool r_w=Position::maze.isFree(ax+1, ay); // right_wall
        bool l_w=Position::maze.isFree(ax-1, ay); // left_wall
        bool d_w=Position::maze.isFree(ax, ay+1); // down_wall
        bool u_w=Position::maze.isFree(ax, ay-1); // upper_wall
        int counter=(int)r_w+(int)l_w+(int)u_w+(int)d_w;
        if(counter ==2 && ((r_w==true and l_w==true and d_w==false and u_w==false )||(r_w==false and l_w==false and d_w==true and u_w==true ))){
            is_corr=true;
        }
        return is_corr;
   }


    std::vector<PositionPtr> children()
    {
        // this method should return  all positions reachable from this one
        std::vector<PositionPtr> generated;

        if(maze.isFree(x+1,y)){
            int i=1;
            while(is_corridor(x+i,y))
               i++;
             generated.push_back(std::make_unique<Position>(x+i,y,i));

        }

        if(maze.isFree(x-1,y)){
            int i=1;
            while(is_corridor(x-i,y))
               i++;
             generated.push_back(std::make_unique<Position>(x-i,y,i));

        }

        if(maze.isFree(x,y+1)){
            int j=1;
            while(is_corridor(x,y+j))
               j++;
             generated.push_back(std::make_unique<Position>(x,y+j,j));

        }
        if(maze.isFree(x,y-1)){
            int j=1;
            while(is_corridor(x,y-j))
               j++;
             generated.push_back(std::make_unique<Position>(x,y-j,j));

        }
        return generated;
    }
protected:
    double distance;
};



int main( int argc, char **argv )
{
    // load file
    std::string filename = "/home/ecn/ecn_arpro/maze/mazes/maze3.png";
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
    Position::maze.saveSolution("maze");
    cv::waitKey(0);
}
