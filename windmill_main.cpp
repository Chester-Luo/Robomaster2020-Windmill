/******************************************
 * Author:  xuelei wei
 * QQ：     2687219372
 ******************************************/

#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
#include "../ImgProcess2021_xuelei/ImageProcess.cpp"
#include "../Position2021_xuelei/Position.cpp"
#include "Windmill.cpp"

using namespace std;
using namespace cv;

int cnnt = 0;

/*************************一些固定参数值*************************/

//线条颜色
Scalar bcolor = Scalar( 255, 0, 0 ); //蓝色
Scalar gcolor = Scalar( 0, 255, 0 ); //绿色
Scalar rcolor = Scalar( 0, 0, 255 ); //红色

//用于预测扇叶装甲板中心到下一个位置的坐标的旋角
//预测部分的预测旋转角theta是要根据实际情况进行确定的（大概是制导组来确定）
double theta = 30;

//装甲板的实际长和宽，用于定位
int armor_length = 195;
int armor_width = 95;

/***************************功能函数声明*************************/

//计算两点间距离
float getDistance ( Point2f pointO, Point2f pointA );

//透视变换：将img图像上位于srcRect[4]坐标上的图像，透视变换到dstRect[4]坐标处
Mat PerspectiveTransform( Mat img, Point2f srcRect[4], Point2f dstRect[4] );

//画出装甲板的区域，p[4]:装甲板的四个顶点, color:线条颜色
void drawArea( Mat img, Point2f p[4], Scalar color );

//拟合圆部分：
//1.获取拟合圆的圆心坐标和半径
bool CircleInfo(std::vector<cv::Point2f>& pts, cv::Point2f& center, float& radius);
//2,返回预测得到的装甲板出现的下一个位置的坐标
Point2f FittingCircle( Mat& img, double theta, int r, vector<Point2f>& circleV, Point2f center );

//自瞄部分：
void AutoAim( Point2f imgCenter, Point2f attackPoint );


/******************************主函数***************************/
int main(){

    cout<<"Pushing 'e' to terminate this program"<<endl;
    //确认能量风车的颜色
    cout<<"Input_Windmill_Color: "<<endl;
    cout<<"0—RED"<<endl;
    cout<<"1—BLUE"<<endl;
    
    int windmill_color;
    cin>>windmill_color;

    Windmill wind_mill( windmill_color ); //为后续的风车扇叶的模板匹配做准备
    Armor winArmor; //为后续的扇叶吃装甲板的识别做准备
    vector<Point2f> armorCenters; //用于计算拟合圆
    ImageProcess imgProcess;
    Monocular posi(armor_length, armor_width);

    VideoCapture cap( "video_example/blue.avi" );
    /*
    VideoCapture cap;
    cap.open( 0 );
    */

    while( cap.isOpened() ){
        Mat srcImg;
        cap>>srcImg;
        
        if( srcImg.empty() ){
            cout<<"video is terminated"<<endl;
            break;
        }
        //imshow( "original_picture", srcImg );
        Mat dstImg = srcImg.clone();

        
        //1.图像预处理部分 
        imgProcess.InputSrcImg( srcImg );
        Mat procImg = imgProcess.Windmill_ImgProcess( windmill_color );
        imshow( "processed_picture", procImg );
        
        //2.风车扇叶识别
        vector< vector<Point> > contours; //存储轮廓点坐标
        vector<Vec4i> hierarchy; //记录轮廓以及相关轮廓的编号
        findContours( procImg, contours, hierarchy, CV_RETR_TREE, CHAIN_APPROX_SIMPLE );
        
        if( hierarchy.size()>0 ){

            RotatedRect tmp_rect;
            for( int i = 0; i > -1; i = hierarchy[i][0] ){ //遍历图像中所有最外侧的轮廓
                
                int child = hierarchy[i][2]; //hierarchy[i][2]表示编号为i的轮廓的子轮廓
                if( child == -1 ) continue; //若child == -1表示当前轮廓i没有子轮廓，不是扇叶
                
                tmp_rect = minAreaRect( contours[i] );
                
                Point2f P[4];
                tmp_rect.points(P); //将矩形的四个点保存在P中

                //为透视变换做准备
                Point2f srcRect[4]; //透视变换前的轮廓外接矩的四个顶点坐标
                Point2f dstRect[4]; //透视变换后的轮廓外接矩的四个顶点坐标

                double width  = getDistance( P[0], P[1] );
                double height = getDistance( P[1], P[2] );
                //矫正提取的叶片的宽高
                if(width>height){
                    srcRect[0]=P[0]; srcRect[1]=P[1];
                    srcRect[2]=P[2]; srcRect[3]=P[3];
                }else{
                    swap(width,height);
                    srcRect[0]=P[1]; srcRect[1]=P[2];
                    srcRect[2]=P[3]; srcRect[3]=P[0];
                }

                //通过面积筛选
                double area=height*width;

                if( area>5000 ){

                    dstRect[0]=Point2f(0,0);        dstRect[1]=Point2f(width,0);
                    dstRect[3]=Point2f(0, height);  dstRect[2]=Point2f(width,height);
                    
                    Mat perspectiveMat = PerspectiveTransform( procImg, srcRect, dstRect );
                    Mat testim = perspectiveMat(Rect(0,0,width,height));
                    //imshow( "testim", testim );
                    if( testim.empty() ){
                        cout<<"filed open"<<endl;
                        continue;
                    }

                    //存储图像：
                    /*Mat storImg;
                    resize( testim, storImg, Size(280, 150) ); //调整存储的模板匹配图像的的大小一致
                    imshow("storImg", storImg);
                    
                    //存储锤子模板图
                    string s = "hammerLeaf"+to_string(cnnt)+".jpg";
                    imwrite("/home/xuelei/RM/上位机_视觉/include_VisualGroup/Windmill2021_xuelei/templateLeaves/hammerLeaves/"+s, storImg);
                    
                    //存储宝剑模板图
                    //string s = "swordLea"+to_string(cnnt)+",jpg"; 
                    //imwrite("/home/xuelei/rm_ws/src/hkvs/src/include_VisualGroup/Windmill2021_xuelei/templateLeaves/swordLeaves"+s, storImg);
                    
                    cnnt++;*/

                    
                    Mat comImg;
                    resize( testim, comImg, Size(280, 150) ); //图像中目标大小与模板大小越相近，匹配结果越准确
                    imshow("comImg", comImg);
                    
                    //判断是否为锤子（即判断是否为待击打的扇叶）
                    if( wind_mill.isHammerLeaf( comImg ) ){
                        
                        //在之前已经有对是否有子轮廓的判断，能到这里就能保证当前轮廓一定是有子轮廓的
                        RotatedRect child_rect = minAreaRect( contours[child] );
                        Point2f child_p[4];
                        child_rect.points( child_p );

                        float width  = child_rect.size.width;
                        float height = child_rect.size.height;

                        if( height > width ) swap(height,width);
                        winArmor.modifyArmorParam(height, width);

                        //进行条件筛选
                        //if( winArmor.isArmor() ){
                            Point2f armor_center = child_rect.center;
                            circle( dstImg, armor_center, height/2, gcolor, 2 );
                            drawArea( dstImg, child_p, gcolor );
                            float angle = 0;

                            posi.InputImagePoints_rotObj2( dstImg, armor_center, child_p[0], child_p[1], child_p[2], child_p[3], child_rect );
                            Point2f armor_next_posi = FittingCircle( dstImg, theta, height/2, armorCenters, armor_center );
                            
                            //自瞄部分：
                            if( armor_next_posi != Point2f(-1, -1) ){
                                Point2f imgCenter = Point2f( dstImg.size().width/2, dstImg.size().height/2 );
                                //line( dstImg, imgCenter, armor_next_posi, gcolor, 2 );
                                AutoAim( imgCenter, armor_next_posi );
                            }
                            
                        //}
                        
                    }

                }
                
            }

        }


        imshow("dstImg", dstImg);
        if( waitKey( 0 ) == 'e' ){
            cout<<"END_PLAY"<<endl;
            break;
        }

    }

    destroyAllWindows();
    return 0;
}


/***************************功能函数主体*************************/

//计算两点间的距离
float getDistance ( Point2f pointO, Point2f pointA ){  
    float distance;  
    distance = powf((pointO.x - pointA.x),2) + powf((pointO.y - pointA.y),2);  
    distance = sqrtf(distance);  
    return distance;  
}

//透视变换：将img图像上位于srcRect[4]坐标上的图像，透视变换到dstRect[4]坐标处
Mat PerspectiveTransform( Mat img, Point2f srcRect[4], Point2f dstRect[4] ){

    //得到透视变换矩阵
    Mat transform = getPerspectiveTransform( srcRect, dstRect ); 
    
    //进行透视变换，perspectMat是透视变换后的图像
    Mat perspectMat;
    warpPerspective( img, perspectMat, transform, img.size() );
    //imshow( "perspectMat", perspectMat );

    return perspectMat;

} 

//画出装甲板的区域，p[4]:装甲板的四个顶点, color:线条颜色
void drawArea( Mat img, Point2f p[4], Scalar color ){
    for( int k = 0; k<4; ++k ){
        line( img, p[k], p[(k+1)%4], color ,2 );
    }
}

/* 拟合圆部分：
 * 通过最小二乘法来拟合圆的信息
 * pts: 所有点坐标
 * center: 得到的圆心坐标
 * radius: 圆的半径
 */
bool CircleInfo(std::vector<cv::Point2f>& pts, cv::Point2f& center, float& radius){
    center = cv::Point2d(0, 0);
    radius = 0.0;
    if (pts.size() < 3) return false;;

    double sumX = 0.0;
    double sumY = 0.0;
    double sumX2 = 0.0;
    double sumY2 = 0.0;
    double sumX3 = 0.0;
    double sumY3 = 0.0;
    double sumXY = 0.0;
    double sumX1Y2 = 0.0;
    double sumX2Y1 = 0.0;
    const double N = (double)pts.size();
    for (int i = 0; i < pts.size(); ++i)
    {
        double x = pts.at(i).x;
        double y = pts.at(i).y;
        double x2 = x * x;
        double y2 = y * y;
        double x3 = x2 *x;
        double y3 = y2 *y;
        double xy = x * y;
        double x1y2 = x * y2;
        double x2y1 = x2 * y;

        sumX += x;
        sumY += y;
        sumX2 += x2;
        sumY2 += y2;
        sumX3 += x3;
        sumY3 += y3;
        sumXY += xy;
        sumX1Y2 += x1y2;
        sumX2Y1 += x2y1;
    }
    double C = N * sumX2 - sumX * sumX;
    double D = N * sumXY - sumX * sumY;
    double E = N * sumX3 + N * sumX1Y2 - (sumX2 + sumY2) * sumX;
    double G = N * sumY2 - sumY * sumY;
    double H = N * sumX2Y1 + N * sumY3 - (sumX2 + sumY2) * sumY;

    double denominator = C * G - D * D;
    if (std::abs(denominator) < DBL_EPSILON) return false;
    double a = (H * D - E * G) / (denominator);
    denominator = D * D - G * C;
    if (std::abs(denominator) < DBL_EPSILON) return false;
    double b = (H * C - E * D) / (denominator);
    double c = -(a * sumX + b * sumY + sumX2 + sumY2) / N;

    center.x = a / (-2);
    center.y = b / (-2);
    radius = std::sqrt(a * a + b * b - 4 * c) / 2;
    return true;
}

//返回预测得到的装甲板出现的下一个位置的坐标
Point2f FittingCircle( Mat& img, double theta, int r, vector<Point2f>& circleV, Point2f center ){
    
    Point2f resPoint = (Point2f)(-1, -1); //预测得到的装甲板中心运动的下一个位置坐标
    Point2f cc; //计算得到的拟合圆圆心

    //在得到装甲板中心点后将其放入缓存队列中
    //用于拟合圆，用30个点拟合圆
    if( circleV.size() < 30 ){
        circleV.push_back( center );
    }else{
        float R;
        //得到拟合的圆心
        CircleInfo( circleV, cc, R );
        circle( img, cc, R, gcolor, 2 );
        //circleV.clear();
    }
    //将打击点围绕圆心旋转某一角度得到预测的打击点
    if( cc.x != 0 && cc.y != 0 ){
        //得到旋转一定角度（这里是30度）后点的位置
        Mat rot_mat = getRotationMatrix2D(cc,theta,1);
        float sinA=rot_mat.at<double>(0,1);//sin(theta);
        float cosA=rot_mat.at<double>(0,0);//cos(theta);
        float xx=-(cc.x-center.x);
        float yy=-(cc.y-center.y);
        resPoint = Point2f( cc.x+cosA*xx-sinA*yy, cc.y+sinA*xx+cosA*yy );
        circle( img, resPoint, r, rcolor, 3 );
    }

    return resPoint;

}

//自瞄部分:
void AutoAim( Point2f imgCenter, Point2f attackPoint ){
    float yaw = attackPoint.x - imgCenter.x;
    float pitch = attackPoint.y - imgCenter.y;
    cout<<"yaw: "<<yaw<<endl;
    cout<<"pitch: "<<pitch<<endl;
    cout<<endl;
}