#include <a_star.h>
#include <maze.h>


using namespace std;
using namespace ecn;

// a node is a x-y position, we move from 1 each time
class Position : public Point
{
    typedef std::unique_ptr<Position> PositionPtr;

protected:
    double distance;


public:
    // constructor from coordinates



    int start_x=Position::maze.start().x;
    int start_y=Position::maze.start().y;
    int end_x=Position::maze.end().x;
    int end_y=Position::maze.end().y;
    Position(int _x, int _y) : Point(_x, _y) {}

    // constructor from base ecn::Point
    Position(ecn::Point p) : Point(p.x, p.y) {}

    //new Constructor
    Position(int _x,int _y, double d): Point(_x,_y){
        this->distance=d;
    }

    int distToParent()
    {
        return this->distance;
    }
    bool is_corridor(int ax, int ay, int &i, int&j){
       bool r_w=Position::maze.isFree(ax+1, ay); // right_wall_checker
       bool l_w=Position::maze.isFree(ax-1, ay); // left_wall_checker
       bool d_w=Position::maze.isFree(ax, ay+1); // down_wall_checker
       bool u_w=Position::maze.isFree(ax, ay-1); // upper_wall_checker
       bool is_corr=false;
       if(i&&(int)maze.isFree(ax+i,ay)+(int)u_w+(int)d_w==1){
              if(u_w==true){
                  j=-1;
                  i=0;
              }
              else if(d_w==true){
                  j=1;
                  i=0;
              }
              else
                  j=j+0; i=i+0;

           is_corr=true;
       }
       else if(j&&(int)maze.isFree(ax,ay+j)+(int)l_w+(int)r_w==1){ //upper_left_corner
               if(l_w==true){
                   i=-1;
                   j=0;
               }
               else if(r_w==true){
                   i=1;
                   j=0;
               }
               else
                   i=i+0; j=j+0;

           is_corr=true;
       }
       return is_corr;
    }





    std::vector<PositionPtr> children()
        {
            // this method should return  all positions reachable from this one
           std::vector<PositionPtr> generated;
           std::vector<pair<int,int>> neighbors{{-1,0},{1,0},{0,-1},{0,1}};
           for(auto [dx, dy]:neighbors){
               int neighbor_x=x+dx;
               int neighbor_y=y+dy;
               if(maze.isFree(neighbor_x,neighbor_y)){
                   while(is_corridor(neighbor_x,neighbor_y,dx,dy) && (neighbor_x!=end_x || neighbor_y!=end_y)){
                         neighbor_x+=dx;
                         neighbor_y+=dy;
                   }
                   generated.push_back(std::make_unique<Position>(neighbor_x,neighbor_y, Point(neighbor_x,neighbor_y).h(Point(x,y),true)));
               }
           }
           return generated;
    }



   void print(const Point &parent)
       {
           std::vector<Point> route;
           std::vector<pair<int,int>> neighbors{{-1,0},{1,0},{0,-1},{0,1}};
           for(auto [dx, dy]:neighbors)
           {
                route.clear();
                int neighbor_x=x+dx;
                int neighbor_y=y+dy;
                if(maze.isFree(neighbor_x,neighbor_y)){
                    route.push_back(Point(neighbor_x,neighbor_y));
                    while(is_corridor(neighbor_x,neighbor_y,dx,dy) && (neighbor_x!=start_x  || neighbor_y!=start_y)){
                        neighbor_x+=dx;
                        neighbor_y+=dy;
                        route.push_back(Point(neighbor_x,neighbor_y));
                    }
                    if(neighbor_x==parent.x && neighbor_y==parent.y)
                    {
                        for(const auto &p:route)
                        {
                            maze.passThrough(p.x,p.y);
                        }
                        break;
                    }

                }
           }
           maze.passThrough(x,y);
       }



    void show(bool closed, const Point &parent)
    {
        const int b = closed?255:0, r = closed?0:255;
        std::vector<Point> route;
        std::vector<pair<int,int>> neighbors{{-1,0},{1,0},{0,-1},{0,1}};
        for(auto [dx, dy]:neighbors){
            int neighbor_x=x+dx;
            int neighbor_y=y+dy;
            if(maze.isFree(neighbor_x,neighbor_y)){
                route.push_back(Point(neighbor_x,neighbor_y));
                while(is_corridor(neighbor_x,neighbor_y,dx,dy) && (neighbor_x!=start_x || neighbor_y!=start_y)){
                    neighbor_x+=dx;
                    neighbor_y+=dy;
                    route.push_back(Point(neighbor_x,neighbor_y));
                }
                if(neighbor_x==parent.x && neighbor_y==parent.y)
                {
                    for(const auto &p:route)
                    {
                        maze.write(p.x,p.y,r,0,b,false);
                    }
                    break;
                }
            }
            maze.write(x,y,r,0,b);
        }

    }



};

int main( int argc, char **argv ) {
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





