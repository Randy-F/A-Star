#include <iostream>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <list>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <cstdlib>
#include <stdio.h>
#include <string>

#define HIGHTH 60
#define WIDTH 100
#define HWIGHT 0.6
#define AWIGHT 0
#define ALLDIRECT 0


namespace myNS
{

    const int kCost1=10; //直移一格消耗
    const int kCost2=14; //斜移一格消耗

    struct Point
    {
        int x,y; //点坐标，这里为了方便按照C++的数组来计算，x代表横排，y代表竖列
        int F,G,H,A; //F=G+H
        Point *parent; //parent的坐标，这里没有用指针，从而简化代码
        Point(int _x,int _y):x(_x),y(_y),F(0),G(0),H(0),A(0),parent(NULL)  //变量初始化
        {
        }
        Point(int _x,int _y,int _A):x(_x),y(_y),F(0),G(0),H(0),A(_A),parent(NULL)  //变量初始化
        {
        }
    };


    class Astar
    {
    public:
        void InitAstar(std::vector<std::vector<int> > &_maze);
        std::list<myNS::Point*> GetPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner,
                                        std::list<Point *> &_openlist, std::list<Point *> &_closelist, std::list<Point *> &_newlist);
    private:
        Point *findPath(Point &startPoint,Point &endPoint,bool isIgnoreCorner);
        std::vector<Point *> getSurroundPoints(const Point *point,bool isIgnoreCorner,Point &direction) const;
        bool isCanreach(const Point *point,const Point *target,bool isIgnoreCorner) const; //判断某点是否可以用于下一步判断
        Point *isInList(const std::list<Point *> &list,const Point *point) const; //判断开启/关闭列表中是否包含某点
        Point *getLeastFpoint(); //从开启列表中返回F值最小的节点
        //计算FGH值
        int calcG(Point *temp_start,Point *point);
        int calcH(Point *point,Point *end);
        int calcF(Point *point);
    private:
        std::vector<std::vector<int> > maze;
        std::list<Point *> openList;  //开启列表
        std::list<Point *> closeList; //关闭列表
        std::list<Point *> newList;
    };


    void Astar::InitAstar(std::vector<std::vector<int> > &_maze)
    {
        maze=_maze;
    }

    int Astar::calcG(Point *temp_start,Point *point)
    {
        int extraG=(abs(point->x-temp_start->x)+abs(point->y-temp_start->y))==1?kCost1:kCost2+point->A*AWIGHT;
        int parentG=point->parent==NULL?0:point->parent->G; //如果是初始节点，则其父节点是空
        return parentG+extraG;
    }

    int Astar::calcH(Point *point,Point *end)
    {
        //用简单的欧几里得距离计算H，这个H的计算是关键，还有很多算法，没深入研究^_^
        return sqrt((double)(end->x-point->x)*(double)(end->x-point->x)+(double)(end->y-point->y)*(double)(end->y-point->y))*kCost1*HWIGHT;
    }

    int Astar::calcF(Point *point)
    {
       // std::cout<<'('<<point->G<<','<<point->H<<','<<point->A*AWIGHT<<')'<<std::endl;
        return point->G+point->H;
    }

    Point *Astar::getLeastFpoint()
    {
        if(!openList.empty())
        {
            auto resPoint=openList.front();
            for(auto &point:openList)
                if(point->F<resPoint->F)
                    resPoint=point;
            return resPoint;
        }
        return NULL;
    }

    Point *Astar::findPath(Point &startPoint,Point &endPoint,bool isIgnoreCorner)
    {
        openList.push_back(new Point(startPoint.x,startPoint.y)); //置入起点,拷贝开辟一个节点，内外隔离
        static uint64 times = 0;
        times++;
        int64 count = -1;


        while(!openList.empty())
        {
            static Point * curPoint=NULL;
            static Point * prePoint=NULL;
            static Point  direction=Point(0,0);

            count++;
            if(times==count)
                return curPoint; //返回列表里的节点指针，不要用原来传入的endpoint指针，因为发生了深拷贝

            prePoint=curPoint;
            curPoint=getLeastFpoint(); //找到F值最小的点

            if (curPoint->parent == NULL)
                direction=Point(6,6);
            else
                direction=Point(curPoint->x-curPoint->parent->x,curPoint->y-curPoint->parent->y);

            openList.remove(curPoint); //从开启列表中删除
            closeList.push_back(curPoint); //放到关闭列表
            //1,找到当前周围八个格中可以通过的格子
            auto surroundPoints=getSurroundPoints(curPoint,isIgnoreCorner,direction);           //HERE!!!!!!!!!!!!!!
            for(auto &target:surroundPoints)
            {                
                if(count+1==times)
                   newList.push_back(target);
                //2,对某一个格子，如果它不在开启列表中，加入到开启列表，设置当前格为其父节点，计算F G H
                if(!isInList(openList,target))
                {
                    target->parent=curPoint;

                    target->G=calcG(curPoint,target);
                    target->H=calcH(target,&endPoint);
                    target->F=calcF(target);

                    openList.push_back(target);
                }
                //3，对某一个格子，它在开启列表中，计算G值, 如果比原来的大, 就什么都不做, 否则设置它的父节点为当前点,并更新G和F
                else
                {
                    int tempG=calcG(curPoint,target);
                    if(tempG<target->G)
                    {
                        target->parent=curPoint;

                        target->G=tempG;
                        target->F=calcF(target);
                    }
                }
           //     Point *resPoint=isInList(openList,&endPoint);          //HERE!!!!     END->CUR??
            //    if(resPoint)
           //         return resPoint; //返回列表里的节点指针，不要用原来传入的endpoint指针，因为发生了深拷贝
          //      std::cout<<"("<<target->x<<","<<target->y<<')'<<std::endl;
            }
        //    std::cout<<std::endl;
         //   std::cout<<"times="<<times<<"count="<<count<<std::endl;

        }

        return NULL;
    }

    std::list<myNS::Point *> Astar::GetPath(Point &startPoint,Point &endPoint,bool isIgnoreCorner,
                                            std::list<Point *> &_openlist, std::list<Point *> &_closelist, std::list<Point *> &_newlist)
    {
        Point *result=findPath(startPoint,endPoint,isIgnoreCorner);
        std::list<Point *> path;
        //返回路径，如果没找到路径，返回空链表
        while(result)
        {
            path.push_front(result);
            result=result->parent;
        }

        _openlist=openList;
        _closelist=closeList;
        _newlist=newList;

        openList.clear();
        closeList.clear();
        newList.clear();

        return path;
    }

    Point *Astar::isInList(const std::list<Point *> &list,const Point *point) const
    {
        //判断某个节点是否在列表中，这里不能比较指针，因为每次加入列表是新开辟的节点，只能比较坐标
        for(auto p:list)
            if(p->x==point->x&&p->y==point->y)
                return p;
        return NULL;
    }

    bool Astar::isCanreach(const Point *point,const Point *target,bool isIgnoreCorner) const
    {
        if(target->x<0||target->x>maze.size()-1
            ||target->y<0&&target->y>maze[0].size()-1
            ||maze[target->x][target->y]==1
            ||target->x==point->x&&target->y==point->y
            ||isInList(closeList,target)) //如果点与当前节点重合、超出地图、是障碍物、或者在关闭列表中，返回false
            return false;
        else
        {
            if(abs(point->x-target->x)+abs(point->y-target->y)==1) //非斜角可以
                return true;
            else
            {
                //斜对角要判断是否绊住
                if(maze[point->x][target->y]==0&&maze[target->x][point->y]==0)
                    return true;
                else
                    return isIgnoreCorner;
            }
        }
    }

    std::vector<Point *> Astar::getSurroundPoints(const Point *point,bool isIgnoreCorner,Point &direction) const
    {
        std::vector<Point *> surroundPoints;

        if ( ALLDIRECT == 1 )
        {
            for(int x=point->x-1;x<=point->x+1;x++)
                for(int y=point->y-1;y<=point->y+1;y++)
                    if(isCanreach(point,new Point(x,y),isIgnoreCorner))
                        surroundPoints.push_back(new Point(x,y));
        }

        else
        {
             if( direction.x<=-2||direction.x>=2||direction.y<=-2||direction.y>=2 )
            {
                for(int x=point->x-1;x<=point->x+1;x++)
                    for(int y=point->y-1;y<=point->y+1;y++)
                        if(isCanreach(point,new Point(x,y),isIgnoreCorner))
                            surroundPoints.push_back(new Point(x,y,0));
            }
            else if( direction.x!=0&&direction.y!=0)
            {
                if(isCanreach(point,new Point(point->x+direction.x,point->y+direction.y),isIgnoreCorner))
                    surroundPoints.push_back(new Point(point->x+direction.x,point->y+direction.y,0));
                if(isCanreach(point,new Point(point->x,point->y+direction.y),isIgnoreCorner))
                    surroundPoints.push_back(new Point(point->x,point->y+direction.y,1));
                if(isCanreach(point,new Point(point->x+direction.x,point->y),isIgnoreCorner))
                    surroundPoints.push_back(new Point(point->x+direction.x,point->y,1));
            }
           else
            {
                if(isCanreach(point,new Point(point->x+direction.x,point->y+direction.y),isIgnoreCorner))
                    surroundPoints.push_back(new Point(point->x+direction.x,point->y+direction.y,0));
                if( direction.x==0 )
                {
                    if(isCanreach(point,new Point(point->x-1,point->y+direction.y),isIgnoreCorner))
                        surroundPoints.push_back(new Point(point->x-1,point->y+direction.y,1));
                    if(isCanreach(point,new Point(point->x+1,point->y+direction.y),isIgnoreCorner))
                        surroundPoints.push_back(new Point(point->x+1,point->y+direction.y,1));
                }
                else
                {
                    if(isCanreach(point,new Point(point->x+direction.x,point->y-1),isIgnoreCorner))
                        surroundPoints.push_back(new Point(point->x+direction.x,point->y-1,1));
                    if(isCanreach(point,new Point(point->x+direction.x,point->y+1),isIgnoreCorner))
                        surroundPoints.push_back(new Point(point->x+direction.x,point->y+1,1));
                }
             }
         }

        return surroundPoints;
    }


}     //myNS

using namespace std;
using namespace cv;

void initMap(std::vector<std::vector<int> > &maze)
{

    for ( int i=0; i<=WIDTH-1; i++ )
        for ( int j=0; j<=HIGHTH-1; j++ )
            if ( i>= 20
                 &&i<=80
                 &&j>=10
                 &&j<=10
                 )
                maze[i][j]=1;
            else if  ( i>= 80
                       &&i<=80
                       &&j>=10
                       &&j<=35
                       )
                maze[i][j]=1;
            else if  ( i>= 55
                       &&i<=80
                       &&j>=35
                       &&j<=35
                       )
                maze[i][j]=1;
            else if  ( i>= 55
                       &&i<=55
                       &&j>=35
                       &&j<=45
                       )
                maze[i][j]=1;
            else if  ( i>= 45
                       &&i<=55
                       &&j>=45
                       &&j<=45
                       )
                maze[i][j]=1;
            else if  ( i>= 45
                       &&i<=45
                       &&j>=35
                       &&j<=45
                       )
                maze[i][j]=1;
            else if  ( i>= 20
                       &&i<=45
                       &&j>=35
                       &&j<=35
                       )
                maze[i][j]=1;
}

void display( std::vector<std::vector<int> > &_maze , list<myNS::Point *> & _path, myNS::Point &_start,myNS::Point &_end,
              std::list<myNS::Point *> &_openlist, std::list<myNS::Point *> &_closelist,std::list<myNS::Point *> &_newlist)
{
    //  setting
    myNS::Point  pixNum(WIDTH  , HIGHTH );   // number of pixel
    myNS::Point  pixScale( 1000/pixNum.x, 600/pixNum.y);
    IplImage *image = cvLoadImage( "/home/randy/slam/navigation/Astar/ubuntu.jpg", -1);   // loading background image

   //    draw pixel
    for ( int i = 0; i < pixNum.x; i++ )
            for ( int j = 0; j < pixNum.y; j++ )
            {
                cv::Point pt1( i*pixScale.x, j*pixScale.y );
                cv::Point pt2( ( (i+1)*pixScale.x ) -1, ( (j+1)*pixScale.y ) -1 );
                   if ( _maze[i][j] == 0)
                    cvRectangle( image, pt1, pt2, CvScalar(200, 200, 200), -1, 8 , 0);
                  else
                    cvRectangle( image, pt1, pt2, CvScalar(100, 100, 100), -1, 8 , 0);
            }

    //  draw path
    cv::Point pt3( 0, 0 );
    cv::Point pt4( 0, 0 );
    int count=0;

    for(auto &p:_path)
    {
       if ( count == 0 )
       {
           pt3 = cvPoint( ( p->x)*pixScale.x+ pixScale.x/2, ( p->y )*pixScale.y + pixScale.y/2);        // x,y iis opposite in cvPoint and vector
           pt4 = cvPoint( ( p->x)*pixScale.x+ pixScale.x/2, ( p->y )*pixScale.y + pixScale.y/2);
       }
       else
       {
           pt3 = pt4;
           pt4 = cvPoint( ( p->x)*pixScale.x+ pixScale.x/2, ( p->y )*pixScale.y + pixScale.y/2);
       }
       count++;

       // cvLine( image, pt3, pt4 , CvScalar(155, 175, 131), 4, 8 , 0);
    }

    for(auto &p:_openlist)
    {
           cv::Point pt7( ( p->x)*pixScale.x, ( p->y)*pixScale.y );
           cv::Point pt8( ( ( p->x+1)*pixScale.x ) -1, ( ( p->y+1)*pixScale.y ) -1 );
           cvRectangle( image, pt7, pt8, CvScalar(58, 38, 3), -1, 8 , 0);
     }

    for(auto &p:_newlist)
    {
           cv::Point pt7( ( p->x)*pixScale.x, ( p->y)*pixScale.y );
           cv::Point pt8( ( ( p->x+1)*pixScale.x ) -1, ( ( p->y+1)*pixScale.y ) -1 );
           cvRectangle( image, pt7, pt8, CvScalar(101, 67, 154), -1, 8 , 0);
     }

    // draw point

    cv::Point pt5 ( _start.x*pixScale.x+ pixScale.x/2, _start.y*pixScale.y+ pixScale.y/2);
    cv::Point pt6 ( _end.x*pixScale.x+ pixScale.x/2, _end.y*pixScale.y+ pixScale.y/2);

    cvCircle( image, pt5, 5 , CvScalar(148, 137, 69), -1, 8 , 0);
    cvCircle( image, pt6, 5 , CvScalar(33, 26, 213),   -1, 8 , 0);

    static int imageCount = 1;

    char itc[10];
    sprintf(itc,"%d.jpg",imageCount );

    cvShowImage("result",image);
    cv::waitKey ( 0 );                  // 暂停程序,等待一个按键输入
    cvSaveImage( itc  , image);

    imageCount++;
}

int main()  
{  
    //初始化地图，用二维矩阵代表地图，1表示障碍物，0表示可通  
    vector<vector<int>> maze(WIDTH, vector<int>(HIGHTH, 0));
    initMap(maze);

    myNS::Astar astar;
    astar.InitAstar(maze);  

    //设置起始和结束点  
    myNS::Point start(70,25);
    myNS::Point end(90,55);
    //A*算法找寻路径  

    std::list<myNS::Point *> openlist,closelist,newlist;

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    while(1)
    {
        list<myNS::Point *> path=astar.GetPath(start,end,false,openlist,closelist,newlist);
        //打印
   //     for(auto &p:newlist)
     //       cout<<'('<<p->x<<','<<p->y<<')'<<endl;

        cout<<endl;
        display(maze, path, start, end, openlist, closelist, newlist);
    }
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
    cout<<"use time："<<time_used.count()<<" seconds."<<endl;

    return 0;  
}  
