//#ifdef _WIN32
//#include <windows.h>
//#endif
//#include <stdio.h>
//#include <stdlib.h>
//#ifndef __APPLE__
//#include <GL/gl.h>
//#include <GL/glut.h>
//#else
//#include <OpenGL/gl.h>
//#include <GLUT/glut.h>
//#endif
//#include <AR/gsub.h>
//#include <AR/video.h>
//#include <AR/param.h>
//#include <AR/ar.h>

//#include "opencv.hpp"
//#include "ImgProc/lineDetector.hpp"
//#include "lsd_opencv.hpp"

//using namespace cv;
//#ifdef _WIN32
//char			*vconf = "Data\\WDM_camera_flipV.xml";
//#else
//char			*vconf = "";
//#endif

//int             xsize, ysize;
//int             thresh = 100;
//int             count = 0;

//int             mode = 1;

//char           *cparam_name    = "/media/windows/pi19404/REPO/Infurnia/ARToolKit/bin/Data/camera_para.dat";
//ARParam         cparam;

//char           *patt_name      = "/media/windows/pi19404/REPO/Infurnia/ARToolKit/bin/Data/android.patt";
//int             patt_id;
//int             patt_width     = 80.0;
//double          patt_center[2] = {0.0, 0.0};
//double          patt_trans[3][4];
//int px,py,pz;
//static void   init(void);
//static void   cleanup(void);
//static void   keyEvent( unsigned char key, int x, int y);
//static void   mainLoop(void);
//static void   draw( double trans[3][4] );
//#undef SOURCE_IMAGE
//#define SOURCE_IMAGE 1
//#ifdef SOURCE_IMAGE
//void process(int ,void *);
//#else
//void run();
//#endif
//bool flag=false;

//void bounds(ARMarkerInfo marker_info)
//{


//}


//int xc,yc,zc;
//Mat frame;
//bool skip=false;
//int main(int argc, char *argv[])
//{


//    glutInit(&argc, argv);
//#ifdef SOURCE_IMAGE
//    frame=imread(argv[1],1);
//    process(0,0);
//    createTrackbar( "T x", "image1",&xc,2000,process);
//    createTrackbar( "T y ","image1",&yc,1000,process);
//    createTrackbar( "T z ","image1",&zc,1000,process);
//    createTrackbar( "threshold", "image1",&thresh,255,process);
//#else
//    cv::namedWindow("image1",1);
//    createTrackbar( "T x", "image1",&xc,2000,0);
//    createTrackbar( "T y ","image1",&yc,1000,0);
//    createTrackbar( "T  z ","image1",&zc,1000,0);
//    createTrackbar( "threshold", "image1",&thresh,255,0);
//    VideoCapture cam;
//    cam = *new VideoCapture(0);
//    if( !cam.isOpened() ) {
//                cout << "Failed to open camera" << endl;
//                return 0;
//            }
//    int key=0;
//    while(key != 27) {
//        cam >> frame;
//        run();
//        key=waitKey(10);
//    }
//#endif
//    cv::waitKey(0);


//}

//struct sort_y
//{
//    bool operator()(Point   const & a, Point const & b)
//    {
//        return a.y <=b.y;
//    }
//};
//struct sort_x
//{
//    bool operator()(Point   const & a, Point const & b)
//    {
//        return a.x <=b.x;
//    }
//};

//class Line
//{
//public:
//    Point p1;
//    Point p2;
//    int length;
//    float m;
//    float c;

//    Line(){};

//    Line(Point p1,Point p2)
//    {
//        this->p1=p1;
//        this->p2=p2;
//        linelength();
//        slope();
//        intercept();
//    }

//    void linelength()
//    {
//        Point diff=p1-p2;
//        length=sqrt(diff.x*diff.x+diff.y*diff.y);
//    }

//    void slope()
//    {

//        m=3.1412/2;
//        if(abs(p1.x-p2.x)>0)
//        m=atan(((p1.y-p2.y)/(p1.x-p2.x)));
//    }

//    void intercept()
//    {
//        if(m!=3.1412/2)
//        c=p1.y-m*p1.x;
//        else
//        c=9999;
//    }

//    float dist_point(Point p3)
//    {
//        float dist=(m*p3.x-p3.y+c/sqrt(m*m+1));
//        return abs(dist);
//    }

//    float dist_line(Line l1)
//    {
//        vector<float> distances;
//        float d1=dist_point(l1.p1);
//        distances.push_back(d1);
//        float d2=dist_point(l1.p2);
//        distances.push_back(d2);
//        float d3=l1.dist_point(p1);
//        distances.push_back(d3);
//        float d4=l1.dist_point(p2);
//        distances.push_back(d4);
//        sort(distances.begin(),distances.end());
//        return distances[3];
//    }
//};




//Line getline(Mat image,ARMarkerInfo tmp)
//{
//    Mat oo;
//    image.copyTo(oo);

//        vector<Point> vertices;
//        for(int i=0;i<4;i++)
//        {
//            vertices.push_back(Point(tmp.vertex[i][0],tmp.vertex[i][1]));
//        }

//        sort(vertices.begin(),vertices.end(),sort_y());

//        Line l=Line(vertices[0],vertices[1]);
//        //computer distance between


//        std::vector<Vec4i> lines;
//        std::vector<double> width, prec, nfa;
//        Ptr<LineSegmentDetector> ls = createLineSegmentDetectorPtr(LSD_REFINE_NONE);


//        Mat i1;
//        cvtColor(image,i1,CV_BGR2GRAY);
//        ls->detect(i1, lines);
//        cv::line(oo,vertices[0],vertices[1],Scalar(0,0,255),3,8);
//        float dist=99999;
//        Line fline;
//        for(int i=0;i<lines.size();i++)
//        {
//            Vec4i p=lines[i];
//            Point p1=Point(p[0],p[1]);
//            Point p2=Point(p[2],p[3]);

//            Line l1=Line(p1,p2);
//            int dslope=abs((l1.m-l.m)*180/3.1412);

//            if(l1.p1.y <=l.p1.y ||l1.p1.y <=l.p2.y||l1.p2.y <=l.p1.y ||l1.p2.y <=l.p2.y)
//            {
//            if(l1.length >=l.length && dslope<30)
//            {
//                float dist1=l.dist_line(l1);
//                if(dist1<dist && dist1>10)
//                {
//                    dist=dist1;
//                    fline=l1;
//                }

//            }
//            }

//        }
//        cv::line(oo,fline.p1,fline.p2,Scalar(0,255,255),1,8);
//        imshow("ooo",oo);
//        return fline;

//}


//#ifdef SOURCE_IMAGE
//void process(int ,void *)
//#else
//void run()
//#endif
//{

//    static int      contF = 0;
//    ARUint8         *dataPtr;
//    ARMarkerInfo    *marker_info;
//    int marker_num=0,count=0;
//    int             j, k;
//    Mat image;




//                k=-1;

//               cv::resize(frame,image,Size(320,240),0,0);
//               cvtColor(image,image,CV_BGR2RGB);
//               cv::normalize(image,image,0,255,cv::NORM_MINMAX);
//               cv::GaussianBlur(image,image,Size(5,5),1,0);


//               xsize=image.cols;
//               ysize=image.rows;
//               if(flag==false)
//               {

//                   flag=true;
//                   init();
//               }

//               dataPtr = (ARUint8 *)image.data;


//                   if( count == 0 ) arUtilTimerReset();
//                   count++;

//                   argDrawMode2D();
//                   argDispImage( dataPtr,0,0);

//                   Mat thresh1;
//                   cv::cvtColor(image,thresh1,CV_RGB2GRAY);
//                   cv::threshold(image,thresh1,thresh,255,CV_THRESH_BINARY);




//                   //thresh=100;
//                   if( arDetectMarker (dataPtr, thresh, &marker_info, &marker_num) < 0 ) {
//                       contF=-1;
//                       skip=true;
//                   }


//                   /*
//                   for(int i=0;i<marker_num;i++)
//                   {
//                       ARMarkerInfo i1=marker_info[i];
//                       cv::circle(image,Point(i1.pos[0],i1.pos[1]),10,Scalar(255,0,0),-1,8);
//                   }
//                   */





//                   k = -1;
//                   for( j = 0; j < marker_num; j++ ) {
//                       ARMarkerInfo tmp=marker_info[j];
//                       for(int i=0;i<4;i++)
//                       {
//                           Point p1=Point(tmp.vertex[(tmp.dir+i)%4][0],tmp.vertex[(tmp.dir+i)%4][1]);
//                           Point p2=Point(tmp.vertex[(tmp.dir+i+1)%4][0],tmp.vertex[(tmp.dir+i+1)%4][1]);
//                           cv::line(image,p1,p2,Scalar(0,255,255),1,8);
//                       }

//                       cerr << patt_id << ":" << marker_info[j].id << endl;
//                       if( patt_id == marker_info[j].id ) {
//                           if( k == -1 ) k = j;
//                           else if( marker_info[k].cf < marker_info[j].cf ) k = j;
//                       }
//                   }

//                    //cerr << marker_num << endl;
//                   if( k == -1 ) {
//                       contF = 0;
//                       skip=true;
//                   }



//                  if(skip==false)
//                  {
//                    if( mode == 0 || contF == 0 ) {
//                        arGetTransMat(&marker_info[k], patt_center, patt_width, patt_trans);
//                       }else{
//                       arGetTransMatCont(&marker_info[k], patt_trans, patt_center, patt_width, patt_trans);
//                   }

//                    Mat m=Mat(3,3,CV_32FC1);



//                    ARMarkerInfo tmp=marker_info[k];


//                    Point center=Point(tmp.pos[0],tmp.pos[1]);
//                    Line l=getline(image,tmp);

//                    vector<Point> vertices;
//                    for(int i=0;i<4;i++)
//                    {
//                        vertices.push_back(Point(tmp.vertex[i][0],tmp.vertex[i][1]));
//                    }

//                    sort(vertices.begin(),vertices.end(),sort_x());

//                    Line l1=Line(vertices[0],vertices[1]);

//                    cv::circle(image,center,5,Scalar(0,0,255),-1,8);
//                    for(int i=0;i<4;i++)
//                    {
//                        Point p1=Point(tmp.vertex[(tmp.dir+i)%4][0],tmp.vertex[(tmp.dir+i)%4][1]);
//                        Point p2=Point(tmp.vertex[(tmp.dir+i+1)%4][0],tmp.vertex[(tmp.dir+i+1)%4][1]);
//                        cv::line(image,p1,p2,Scalar(0,255,255),1,8);
//                    }

//                    cv::line(image,l1.p1,l1.p2,Scalar(0,0,255),5,8);
//                    cerr << tmp.dir << "DDDDDDDd" <<endl;
//      //              cerr << center.y <<":" << l.p1.y <<":" << l.p2.y << endl;


//                    Point dp;
//                    if(l.p1.y>l.p2.y)
//                    {
//                        dp=center-l.p1;
//                        py=(patt_width/4)+(patt_width*std::max((int)l.dist_point(center),dp.y)/l1.length);
//                    }
//                    else
//                    {

//                        dp=center-l.p2;
//                        py=(patt_width/4)+(patt_width*std::max((int)l.dist_point(center),dp.y)/l1.length) ;

//                    }


//                    int tc=0;
//                    cerr << tmp.dir << "LLL" <<yc <<":" << 20+dp.y*80/l1.length<< endl;

//                    //Line l=Line(Point(tmp.vertex[tmp.dir][0],tmp.vertex[tmp.dir][1]);

//                    //if(tmp.dir==3 || tmp.dir==2 ||tmp.dir==1||)
//                    {
//                        /*  tc=py-40;
//                          if(tmp.dir==3)
//                          {
//                          px=-xc+1000/2;
//                          py=-py-40;
//                          }
//                          else if(tmp.dir==2)
//                          {
//                          px=-xc+1000/2;
//                          py=tc;
//                          }
//                          else if(tmp.dir==1)
//                          {
//                            py=py-40;
//                            px=xc-1000/2;
//                          }
//                          */
//                        px=xc-1000/2;
//                        py=yc-1000/2;

//                          for(int k=0;k<2;k++)
//                          {
//                          int index=(4-tmp.dir+k)%4;
//                          cv::circle(image,Point(tmp.vertex[index][0],tmp.vertex[index][1]),10,Scalar(0,255),-1,8);
//                          k++;
//                          index=(4-tmp.dir+k)%4;
//                          cv::circle(image,Point(tmp.vertex[0][0],tmp.vertex[0][1]),10,Scalar(255,0,0),-1,8);
//                          }

//                    }
//                    /*
//                    if(tmp.dir==1||tmp.dir==0)
//                    {
//                        if(tmp.dir==2)
//                        {
//                        tc=-yc+50;
//                        px=-xc+1000/2;
//                        }
//                        else
//                        {
//                        tc=yc-50;
//                        px=-xc+1000/2;
//                        }
//                        py=tc;

//                        //xc=tc;
//                    }
//                    */

//                  }

//                   //cvtColor(image,image,CV_RGB2BGR);
//                   imshow("image1",image);
//                   imshow("image2",thresh1);
//                   draw( patt_trans );
//                   argSwapBuffers();
//                   cv::waitKey(10);

//                   skip=false;
//           }



//static void init( void )
//{
//    ARParam  wparam;


//    /* set the initial camera parameters */
//    if( arParamLoad(cparam_name, 1, &wparam) < 0 ) {
//        printf("Cam era parameter load error !!\n");
//        exit(0);
//    }
//    arParamChangeSize( &wparam, xsize, ysize, &cparam );
//    arInitCparam( &cparam );
//    //printf("*** Camera Parameter ***\n");
//    //arParamDisp( &cparam );

//    if( (patt_id=arLoadPatt(patt_name)) < 0 ) {
//        printf("pattern load error !!\n");
//        exit(0);
//    }

//    /* open the graphics window */
//    argInit( &cparam, 1.0, 0, 0, 0, 0 );
//}

///* cleanup function called when program exits */
//static void cleanup(void)
//{
//    argCleanup();
//}

//static void draw( double trans[3][4] )
//{
//    double    gl_para[16];
//    GLfloat   mat_ambient[]     = {0.0, 0.0, 1.0, 1.0};
//    GLfloat   mat_flash[]       = {0.0, 0.0, 1.0, 1.0};
//    GLfloat   mat_flash_shiny[] = {50.0};
//    GLfloat   light_position[]  = {100.0,-200.0,200.0,0.0};
//    GLfloat   ambi[]            = {0.1, 0.1, 0.1, 0.1};
//    GLfloat   lightZeroColor[]  = {0.9, 0.9, 0.9, 0.1};

//    argDrawMode3D();
//    argDraw3dCamera( 0, 0 );
//    glClearDepth( 1.0 );
//    glClear(GL_DEPTH_BUFFER_BIT);
//    glEnable(GL_DEPTH_TEST);
//    glDepthFunc(GL_LEQUAL);

//    /* load the camera transformation matrix */
//    argConvGlpara(trans, gl_para);
//    glMatrixMode(GL_MODELVIEW);
//    glLoadMatrixd( gl_para );

//    glEnable(GL_LIGHTING);
//    glEnable(GL_LIGHT0);
//    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
//    glLightfv(GL_LIGHT0, GL_AMBIENT, ambi);
//    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightZeroColor);
//    glMaterialfv(GL_FRONT, GL_SPECULAR, mat_flash);
//    glMaterialfv(GL_FRONT, GL_SHININESS, mat_flash_shiny);
//    glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
//    glMatrixMode(GL_MODELVIEW);
//    //change the z coordinate so that it is always on the ground 25
//    //x and y correspond to the actual dimensions wrt the corner of the marker
//    //to restrict it to a line in the image,find the inverse transformation
//    //a point in image will lie at what position in the real world
//    //then bound it to the
//    int size=30;
//    //yc=yc-(size);
//    //xc=xc;//+(size/2);
//    //yc=yc;
//    //xc=xc-1000/2;
///*

//    int pointx[]={xc,yc,zc,1};
//    int res[4];
//    for(int j=0;j<3;j++)
//    {
//           res[j]=0;

//    for(int i=0;i<4;i++)
//    {
//            res[j]=res[j]+trans[j][i]*pointx[i];
//    }
//    }
//*/
//    glTranslatef(px,py,size/2);//world co-ordinate to pixel co-ordinate

//    //0,0 corresponds to the bottom right corner of the marker
//    glutSolidCube(size);

//    glTranslatef(px+10,py+10,size/2);//world co-ordinate to pixel co-ordinate

//    glutSolidCube(size);
//    glDisable( GL_LIGHTING );

//    glDisable( GL_DEPTH_TEST );
//}
