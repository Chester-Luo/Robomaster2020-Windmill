#include "Windmill.h"

/********************************Armor************************************/
Armor::Armor( float height, float width ){
    this->height = height;
    this->width = width;
    this->ratio_HW = this->height / this->width;
    this->area = this->height * this->width;
}

void Armor::modifyArmorParam( float height, float width ){
    this->height = height;
    this->width = width;
    this->ratio_HW = this->height / this->width;
    this->area = this->height * this->width;
}

bool Armor::isArmor(){
    return ( this->ratio_HW <= this->maxHWRatio && this->area <= this->maxArea && this->area >= this->minArea );
}


/*********************************Windmill**************************************/

//默认构造函数，导入扇叶匹配的扇叶模板图像
Windmill::Windmill( int color ){
    this->windmill_color = color;
    for( int i = 0; i<tmpHamNum; ++i ){ //未被击打过的扇叶模板
        tmpl_hammerLeaves[i] = imread( HammerLeaves_File_Path + to_string(i) + Picture_Kind, IMREAD_GRAYSCALE );
        cout<<i<<"width: "<<tmpl_hammerLeaves[i].size().width<<", "<<"height: "<<tmpl_hammerLeaves[i].size().height<<endl;
    }
    for( int i = 0; i<tmpSwoNum; ++i ){ //击打过的扇叶模板
        tmpl_swordLeaves[i] = imread( SwordLeaves_File_Path + to_string(i) + Picture_Kind, IMREAD_GRAYSCALE );
        cout<<i<<"width: "<<tmpl_swordLeaves[i].size().width<<", "<<"height: "<<tmpl_swordLeaves[i].size().height<<endl;
    }
}

//清空向量 hammer_value sword_value 内存储的匹配值
void Windmill::clearHamSwdValue(){
    this->hammer_value.clear();
    this->sword_value.clear();
}

//对传入的图像与先前导入的模板图进行模板匹配，并返回匹配值
double Windmill::WinMatchTemplate( Mat img, Mat tmpl_leaf, Point& matchPosi, int method ){
            
    double value = 0;
    int cols = img.cols - tmpl_leaf.cols + 1;
    int rows = img.rows - tmpl_leaf.rows + 1;
    Mat result = Mat( cols, rows, CV_32FC1 );
    matchTemplate( img, tmpl_leaf, result, method );

    double minVal, maxVal;
    Point minLoc, maxLoc;
    minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

    switch(method){

        case CV_TM_SQDIFF:
        case CV_TM_SQDIFF_NORMED:
            matchPosi = minLoc;
            value = minVal;

        default:
            matchPosi = maxLoc;
            value = maxVal;

    }

    return value;
}


//判断扇叶是否为锤子扇叶（即待打击扇叶）
bool Windmill::isHammerLeaf( Mat img ){
    
    Point matchPosi;
    double value;

    for( int i = 0; i < tmpHamNum ; ++i ){
        value = WinMatchTemplate( img, tmpl_hammerLeaves[i], matchPosi, CV_TM_CCOEFF_NORMED);
        this->hammer_value.push_back(value);
    }

    for( int i = 0; i < tmpSwoNum; ++i ){
        value = WinMatchTemplate( img, tmpl_swordLeaves[i], matchPosi, CV_TM_CCOEFF_NORMED);
        this->sword_value.push_back(value);
    }

    //与锤子扇叶模板匹配的最大值编号
    int hammer_maxv = 0;
    for( int t1 = 1; t1 < tmpHamNum; t1++ ){
        if( this->hammer_value[t1] > this->hammer_value[hammer_maxv]){
            hammer_maxv=t1;
        }
    }

    //与宝剑扇叶模板匹配的最大值编号
    int sword_maxv = 0;
    for( int t2 = 1; t2 < tmpSwoNum; t2++ ){
        if( this->sword_value[t2] > this->sword_value[sword_maxv] ){
            sword_maxv=t2;
        }
    }

    cout<<"hammer: "<<this->hammer_value[hammer_maxv]<<endl;
    cout<<"sword: "<<this->sword_value[sword_maxv]<<endl;

    /*
        是锤子扇叶的条件：
            1.首先保证图像是个锤子即 hammer_value[hammer_maxv] > sword_value[sword_maxv]
            2.保证锤子模板匹配最大值要>0.6即 hammer_value[hammer_maxv]>0.6
    */
    bool result = ( ( this->hammer_value[hammer_maxv] > this->sword_value[sword_maxv] ) && this->hammer_value[hammer_maxv] > 0.8 );
    
    //一定要清空向量 hammer_value sword_value 内的值，防止下次调用此函数时，先前存储的数据造成干扰
    clearHamSwdValue();

    return result;

}

