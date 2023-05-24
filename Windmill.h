#ifndef WINDMILL_H
#define WINDMILL_H

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

//注意：模板匹配的图片读取路径要为绝对路径，否则无法读取到
#define HammerLeaves_File_Path "templateLeaves/hammerLeaves/hammerLeaf"
#define SwordLeaves_File_Path  "templateLeaves/swordLeaves/swordLeaf"
#define Picture_Kind ".jpg"

#define tmpHamNum 6 //模板匹配中锤子数量
#define tmpSwoNum 4 //模板匹配中宝剑数量

class Armor{
    public:
        float height;
        float width;
        float ratio_HW; //装甲板高宽比
        float area;
        
        const float maxHWRatio=0.7153846;
        const float maxArea=2000;
        const float minArea=500;
        
    public:

        //默认构造函数
        Armor(){}

        //构造函数
        Armor( float height, float width );

        //修改装甲板的高、宽
        void modifyArmorParam( float height, float width );

        //判断当前的参数是否满足是装甲板的条件，即判断是不是装甲板
        bool isArmor();

};

class Windmill{

    public:
        int windmill_color;                    //记录风车颜色（0-RED， 1-BLUE）
        Mat tmpl_hammerLeaves[tmpHamNum];      //用于存储模板匹配所需要的宝剑模板图
        Mat tmpl_swordLeaves[tmpSwoNum];       //用于存储模板匹配所需要的锤子模板图
        vector<double> hammer_value;           //用于存储进行模板匹配后的匹配值——针对锤子（即待打击扇叶）
        vector<double> sword_value;            //用于存储进行模板匹配后的匹配值——针对宝剑（即已打击过的扇叶）

    public:

        //构造函数，确认扇叶颜色，导入扇叶匹配的扇叶模板图像
        Windmill( int color );

        //清空向量 hammer_value sword_value 内存储的匹配值
        void clearHamSwdValue();

        //对传入的图像与先前导入的模板图进行模板匹配，并返回匹配值
        double WinMatchTemplate( Mat img, Mat tmpl_leaf, Point& matchPosi, int method );

        //判断扇叶是否为锤子扇叶（即待打击扇叶）
        bool isHammerLeaf( Mat img );
        
};


#endif