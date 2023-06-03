#include "10_home_script.h"
CHomeScript::CHomeScript()
{
    
}

CHomeScript::~CHomeScript()
{

}

void CHomeScript::Queue()
{
    stAct newAct;

    newAct.nAct = ACT_SPEAK;
    newAct.strTarget = "科大讯飞语音交互 Demo";
    newAct.nDuration = 20;
    arAct.push_back(newAct);

    newAct.nAct = ACT_LISTEN;
    newAct.strTarget = "机器人";
    newAct.nDuration = 5;
    arAct.push_back(newAct);

    newAct.nAct = ACT_SPEAK;
    newAct.strTarget = "你好,已经准备好了,主人";
    newAct.nDuration = 5;
    arAct.push_back(newAct);

    // newAct.nAct = ACT_LISTEN;
    // newAct.strTarget = "我渴了";
    // newAct.nDuration = 5;
    // arAct.push_back(newAct);

    // newAct.nAct = ACT_SPEAK;
    // newAct.strTarget = "好的,主人你想来瓶饮料吗";
    // newAct.nDuration = 5;
    // arAct.push_back(newAct);

    // newAct.nAct = ACT_LISTEN;
    // newAct.strTarget = "是的";
    // newAct.nDuration = 5;
    // arAct.push_back(newAct);

    newAct.nAct = ACT_LISTEN;
    newAct.strTarget = "饮料";//饮料是主关键词，给我去客厅取一杯饮料，如果没有听到房间名，那么默认前往客厅
    newAct.nDuration = 5;
    arAct.push_back(newAct);

    newAct.nAct = ACT_SPEAK;
    newAct.strTarget = "好的,这就给您去取";
    newAct.nDuration = 5;
    arAct.push_back(newAct);

    newAct.nAct = ACT_GOTO;
    newAct.strTarget = "meeting_observice";
    arAct.push_back(newAct);

    newAct.nAct = ACT_FIND_OBJ;
    newAct.strTarget = "";
    arAct.push_back(newAct);


    // newAct.nAct = ACT_SPEAK;
    // newAct.strTarget = "开始抓取饮料";
    // newAct.nDuration = 5;
    // arAct.push_back(newAct);

    // newAct.nAct = ACT_GRAB;
    // newAct.strTarget = "";
    // newAct.nDuration = 5;
    // arAct.push_back(newAct);

    newAct.nAct = ACT_SPEAK;
    newAct.strTarget = "到达抓取地点，开始抓取                                                                     饮料抓取完毕    返回原位";
    newAct.nDuration = 6;
    arAct.push_back(newAct);

    newAct.nAct = ACT_GOTO;
    newAct.strTarget = "origin";
    arAct.push_back(newAct);

    newAct.nAct = ACT_SPEAK;
    newAct.strTarget = "饮料已取回,主人请慢用";
    newAct.nDuration = 5;
    arAct.push_back(newAct);

    // newAct.nAct = ACT_PASS;
    // newAct.strTarget = "";
    // newAct.nDuration = 5;
    // arAct.push_back(newAct);

    newAct.nAct = ACT_SPEAK;
    newAct.strTarget = "任务完成";
    newAct.nDuration = 5;
    arAct.push_back(newAct);
}