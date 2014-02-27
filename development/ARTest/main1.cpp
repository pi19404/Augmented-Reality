#ifdef _WIN32
#include <windows.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#ifndef __APPLE__
#include <GL/gl.h>
#include <GL/glut.h>
#else
#include <OpenGL/gl.h>
#include <GLUT/glut.h>
#endif
#include <AR/gsub.h>
#include <AR/video.h>
#include <AR/param.h>
#include <AR/ar.h>

#include "opencv.hpp"
#include "ImgProc/lineDetector.hpp"
#include "lsd_opencv.hpp"

using namespace cv;
#ifdef _WIN32
char			*vconf = "Data\\WDM_camera_flipV.xml";
#else
char			*vconf = "";
#endif

int             xsize, ysize;
int             thresh = 100;
int             count = 0;

int             mode = 1;

char           *cparam_name    = "/media/windows/pi19404/REPO/Infurnia/ARToolKit/bin/Data/camera_para.dat";
ARParam         cparam;

char           *patt_name      = "/media/windows/pi19404/REPO/Infurnia/ARToolKit/bin/Data/android.patt";
int             patt_id;
int             patt_width     = 80.0;
double          patt_center[2] = {0.0, 0.0};
double          patt_trans[3][4];
int px,py,pz;
static void   init(void);
static void   cleanup(void);
static void   keyEvent( unsigned char key, int x, int y);
static void   mainLoop(void);
static void   draw( double trans[3][4] );
#undef SOURCE_IMAGE
#define SOURCE_IMAGE 1
#ifdef SOURCE_IMAGE
void process(int ,void *);
#else
void run();
#endif
bool flag=false;

void bounds(ARMarkerInfo marker_info)
{


}


int xc,yc,zc;
Mat frame;
bool skip=false;
int main(int argc, char *argv[])
{


    glutInit(&argc, argv);
#ifdef SOURCE_IMAGE
    frame=imread(argv[1],1);
    process(0,0);
    createTrackbar( "T x", "image1",&xc,2000,process);
    createTrackbar( "T y ","image1",&yc,1000,process);
    createTrackbar( "T z ","image1",&zc,1000,process);
    createTrackbar( "threshold", "image1",&thresh,255,process);
#else
    cv::namedWindow("image1",1);
    createTrackbar( "T x", "image1",&xc,2000,0);
    createTrackbar( "T y ","image1",&yc,1000,0);
    createTrackbar( "T  z ","image1",&zc,1000,0);
    createTrackbar( "threshold", "image1",&thresh,255,0);
    VideoCapture cam;
    cam = *new VideoCapture(0);
    if( !cam.isOpened() ) {
                cout << "Failed to open camera" << endl;
                return 0;
            }
    int key=0;
    while(key != 27) {
        cam >> frame;
        run();
        key=waitKey(10);
    }
#endif
    cv::waitKey(0);


}

struct sort_y
{
    bool operator()(Point   const & a, Point const & b)
    {
        return a.y <=b.y;
    }
};
struct sort_x
{
    bool operator()(Point   const & a, Point const & b)
    {
        return a.x <=b.x;
    }
};

class Line
{
public:
    Point p1;
    Point p2;
    int length;
    float m;
    float c;

    Line(){};

    Line(Point p1,Point p2)
    {
        this->p1=p1;
        this->p2=p2;
        linelength();
        slope();
        intercept();
    }

    void linelength()
    {
        Point diff=p1-p2;
        length=sqrt(diff.x*diff.x+diff.y*diff.y);
    }

    void slope()
    {

        m=3.1412/2;
        if(abs(p1.x-p2.x)>0)
        m=atan(((p1.y-p2.y)/(p1.x-p2.x)));
    }

    void intercept()
    {
        if(m!=3.1412/2)
        c=p1.y-m*p1.x;
        else
        c=9999;
    }

    float dist_point(Point p3)
    {
        float dist=(m*p3.x-p3.y+c/sqrt(m*m+1));
        return abs(dist);
    }

    float dist_line(Line l1)
    {
        vector<float> distances;
        float d1=dist_point(l1.p1);
        distances.push_back(d1);
        float d2=dist_point(l1.p2);
        distances.push_back(d2);
        float d3=l1.dist_point(p1);
        distances.push_back(d3);
        float d4=l1.dist_point(p2);
        distances.push_back(d4);
        sort(distances.begin(),distances.end());
        return distances[3];
    }
};




Line getline(Mat image,ARMarkerInfo tmp)
{
    Mat oo;
    image.copyTo(oo);

        vector<Point> vertices;
        for(int i=0;i<4;i++)
        {
            vertices.push_back(Point(tmp.vertex[i][0],tmp.vertex[i][1]));
        }

        sort(vertices.begin(),vertices.end(),sort_y());

        Line l=Line(vertices[0],vertices[1]);
        //computer distance between


        std::vector<Vec4i> lines;
        std::vector<double> width, prec, nfa;
        Ptr<LineSegmentDetector> ls = createLineSegmentDetectorPtr(LSD_REFINE_NONE);


        Mat i1;
        cvtColor(image,i1,CV_BGR2GRAY);
        ls->detect(i1, lines);
        cv::line(oo,vertices[0],vertices[1],Scalar(0,0,255),3,8);
        float dist=99999;
        Line fline;
        for(int i=0;i<lines.size();i++)
        {
            Vec4i p=lines[i];
            Point p1=Point(p[0],p[1]);
            Point p2=Point(p[2],p[3]);

            Line l1=Line(p1,p2);
            int dslope=abs((l1.m-l.m)*180/3.1412);

            if(l1.p1.y <=l.p1.y ||l1.p1.y <=l.p2.y||l1.p2.y <=l.p1.y ||l1.p2.y <=l.p2.y)
            {
            if(l1.length >=l.length && dslope<30)
            {
                float dist1=l.dist_line(l1);
                if(dist1<dist && dist1>10)
                {
                    dist=dist1;
                    fline=l1;
                }

            }
            }

        }
        cv::line(oo,fline.p1,fline.p2,Scalar(0,255,255),1,8);
        imshow("ooo",oo);
        return fline;

}


int hamming_dist(Mat t1,Mat t2)
{
    int dist=0;
    //computing the hammming distance

//printf(":::::::::::: \n\n\n");
    for (int y=0;y<4;y++)
    {
        int minSum=1e5;
        //hamming distance to each possible word
        for (int p=0;p<4;p++)
        {
            int sum=0;
            //now, count
            for (int x=0;x<4;x++)
            {
                //printf("%d %d : ",t1.at<uchar>(y,x),t2.at<uchar>(p,x));
                sum+=(t1.at<uchar>(y,x) == t2.at<uchar>(p,x))?0:1;
            }
            //printf("\n") ;
            //printf("%d ",sum);
            if (minSum>sum) minSum=sum;
        }
        //printf(":::::::::::: \n\n\n");
        //do the and
        dist+=minSum;
    }

    return dist;
}

//rorate bitwise pattern in anticlockwise direction
Mat rotateB(Mat t)
{
    Mat out;
    t.copyTo(out);
    for (int i=0;i<t.rows;i++)
    {
        for (int j=0;j<t.cols;j++)
        {
            out.at<uchar>(i,j)=t.at<uchar>(t.cols-j-1,i);
        }
    }
    return out;

}


#ifdef SOURCE_IMAGE
void process(int ,void *)
#else
void run()
#endif
{

    static int      contF = 0;
    ARUint8         *dataPtr;
    ARMarkerInfo    *marker_info;
    int marker_num=0,count=0;
    int             j, k;
    Mat image;




                k=-1;

               cv::resize(frame,image,Size(320,240),0,0);
               cvtColor(image,image,CV_BGR2RGB);
               cv::normalize(image,image,0,255,cv::NORM_MINMAX);
               cv::GaussianBlur(image,image,Size(5,5),1,0);


               xsize=image.cols;
               ysize=image.rows;
               if(flag==false)
               {

                   flag=true;
                   init();
               }

               dataPtr = (ARUint8 *)image.data;


                   if( count == 0 ) arUtilTimerReset();
                   count++;

                   argDrawMode2D();
                   argDispImage( dataPtr,0,0);

                   Mat thresh1;
                   cv::cvtColor(image,thresh1,CV_RGB2GRAY);
                   cv::threshold(image,thresh1,thresh,255,CV_THRESH_BINARY);




                   //thresh=100;
                   if( arDetectMarker (dataPtr, thresh, &marker_info, &marker_num) < 0 ) {
                       contF=-1;
                       skip=true;
                   }




                   //selecting the best marker
                   k = -1;
                   for( j = 0; j < marker_num; j++ ) {
                       ARMarkerInfo tmp=marker_info[j];
                       for(int i=0;i<4;i++)
                       {
                           Point p1=Point(tmp.vertex[(tmp.dir+i)%4][0],tmp.vertex[(tmp.dir+i)%4][1]);
                           Point p2=Point(tmp.vertex[(tmp.dir+i+1)%4][0],tmp.vertex[(tmp.dir+i+1)%4][1]);
                           cv::line(image,p1,p2,Scalar(0,255,255),1,8);
                       }

                       cerr << patt_id << ":" << marker_info[j].id << endl;
                       if( patt_id == marker_info[j].id ) {
                           if( k == -1 ) k = j;
                           else if( marker_info[k].cf < marker_info[j].cf ) k = j;
                       }
                   }


                   if(k!=-1)
                   {
                   //recognize markers
                   ARMarkerInfo tmp=marker_info[k];

                   vector<Point2f> marker_points;
                   marker_points.push_back(Point2f(0,0));
                   marker_points.push_back(Point2f(patt_width-1,0));
                   marker_points.push_back(Point2f(patt_width-1,patt_width-1));
                   marker_points.push_back(Point2f(0,patt_width-1));

                   vector<Point2f> detected_points;
                   for(int i=0;i<4;i++)
                   {
                       detected_points.push_back(Point2f(tmp.vertex[(4-tmp.dir+i)%4][0],tmp.vertex[(4-tmp.dir+i)%4][1]));
                   }
                   cv::Mat markerTransform = cv::getPerspectiveTransform(detected_points,marker_points);
                   Mat image2;
                   cv::warpPerspective(image, image2,  markerTransform,Size(patt_width,patt_width));

                   imshow("can",image2);
                   cvtColor(image2,image2,CV_BGR2GRAY);
                   //cv::threshold(image2,image2,0,255,CV_THRESH_OTSU);
                   //cerr << image2.cols <<":" << image2.rows << endl;


                   Mat bitx=Mat(4,4,CV_8UC1);
                   for(int i=0;i<8;i++)
                   {
                       for(int j=0;j<8;j++)
                       {
                           Rect r=Rect(j*10,i*10,10,10);
                           Mat roi=image2(r);
                           int cnt=cv::countNonZero(roi);
                           if((i>1 && i<6)&&(j>1 && j<6))
                           {
                           if(cnt > 10*10/2)
                           {
                                   bitx.at<uchar>(i-2,j-2)=1;
                                   //printf("%d ",1);

                           }
                           else
                           {
                                   bitx.at<uchar>(i-2,j-2)=0;
                                 //  printf("%d ",0);
                           }
                          }

                   }
                       //printf("\n");
                }


                   Mat tempx=Mat(4,4,CV_8UC1);
                   tempx.at<uchar>(0,0)=1;
                   tempx.at<uchar>(1,0)=1;
                   tempx.at<uchar>(2,0)=1;
                   tempx.at<uchar>(3,0)=1;
                   tempx.at<uchar>(0,1)=1;
                   tempx.at<uchar>(1,1)=1;
                   tempx.at<uchar>(2,1)=0;
                   tempx.at<uchar>(3,1)=1;
                   tempx.at<uchar>(0,2)=1;
                   tempx.at<uchar>(1,2)=0;
                   tempx.at<uchar>(2,2)=0;
                   tempx.at<uchar>(3,2)=1;
                   tempx.at<uchar>(0,3)=1;
                   tempx.at<uchar>(1,3)=1;
                   tempx.at<uchar>(2,3)=1;
                   tempx.at<uchar>(3,3)=1;



                   printf("\n\n");
                   for(int i=0;i<4;i++)
                   {
                       for(int j=0;j<4;j++)
                       {
                           printf("%d ",tempx.at<uchar>(i,j));
                       }
                       printf("\n");
                   }



                   int dist=0;
                   int dist1=0;
                   int index=0;
                   int minDist=9999;
                   Mat comp;

                   bitx.copyTo(comp);
                   for(int kx=0;kx<4;kx++)
                   {

                        dist=hamming_dist(comp,tempx);
                        dist1=hamming_dist(comp.t(),tempx.t());
                        dist=max(dist,dist1);
                        if(dist<minDist)
                        {
                            minDist=dist;
                            index=kx;
                        }
                        comp=rotateB(comp);
                   }


                }





                  //estimating the marker rotation so that proper points can be given for pose estimation
                  //also this step provides further validation for the marker
                  //identifying the marker code





                  if(skip==false)
                  {
                    if( mode == 0 || contF == 0 ) {
                        arGetTransMat(&marker_info[k], patt_center, patt_width, patt_trans);
                       }else{
                       arGetTransMatCont(&marker_info[k], patt_trans, patt_center, patt_width, patt_trans);
                   }
                  }

                   //cvtColor(image,image,CV_RGB2BGR);
                   imshow("image1",image);
                   imshow("image2",thresh1);
                   draw( patt_trans );
                   argSwapBuffers();
                   cv::waitKey(10);

                   skip=false;
           }



static void init( void )
{
    ARParam  wparam;


    /* set the initial camera parameters */
    if( arParamLoad(cparam_name, 1, &wparam) < 0 ) {
        printf("Cam era parameter load error !!\n");
        exit(0);
    }
    arParamChangeSize( &wparam, xsize, ysize, &cparam );
    arInitCparam( &cparam );
    //printf("*** Camera Parameter ***\n");
    //arParamDisp( &cparam );

    if( (patt_id=arLoadPatt(patt_name)) < 0 ) {
        printf("pattern load error !!\n");
        exit(0);
    }

    /* open the graphics window */
    argInit( &cparam, 1.0, 0, 0, 0, 0 );
}

/* cleanup function called when program exits */
static void cleanup(void)
{
    argCleanup();
}

static void draw( double trans[3][4] )
{
    double    gl_para[16];
    GLfloat   mat_ambient[]     = {0.0, 0.0, 1.0, 1.0};
    GLfloat   mat_flash[]       = {0.0, 0.0, 1.0, 1.0};
    GLfloat   mat_flash_shiny[] = {50.0};
    GLfloat   light_position[]  = {100.0,-200.0,200.0,0.0};
    GLfloat   ambi[]            = {0.1, 0.1, 0.1, 0.1};
    GLfloat   lightZeroColor[]  = {0.9, 0.9, 0.9, 0.1};

    argDrawMode3D();
    argDraw3dCamera( 0, 0 );
    glClearDepth( 1.0 );
    glClear(GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    /* load the camera transformation matrix */
    argConvGlpara(trans, gl_para);
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixd( gl_para );

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambi);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightZeroColor);
    glMaterialfv(GL_FRONT, GL_SPECULAR, mat_flash);
    glMaterialfv(GL_FRONT, GL_SHININESS, mat_flash_shiny);
    glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
    glMatrixMode(GL_MODELVIEW);
    //change the z coordinate so that it is always on the ground 25
    //x and y correspond to the actual dimensions wrt the corner of the marker
    //to restrict it to a line in the image,find the inverse transformation
    //a point in image will lie at what position in the real world
    //then bound it to the
    int size=30;

    glTranslatef(0,0,size/2);//world co-ordinate to pixel co-ordinate

    //0,0 corresponds to the bottom right corner of the marker
    glutSolidCube(size);

    //glTranslatef(px+10,py+10,size/2);//world co-ordinate to pixel co-ordinate

    //glutSolidCube(size);
    glDisable( GL_LIGHTING );

    glDisable( GL_DEPTH_TEST );
}

