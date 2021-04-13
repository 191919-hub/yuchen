#ifndef __PROGRAM_CFG__
#define __PROGRAM_CFG__


#define LC_DISP_RESOLUTION_0D1   1    //冷藏显示温度分辨率为0.1度，屏蔽本宏或者定义为0，则分辨率恢复为1
#define  FORBID_BUZZ  1   //禁止蜂鸣器响  调试时可以禁止蜂鸣器响，


//冷藏设定温度上限、下限       2~16之间（河南项目 周金波要求）  之前282功能书要求5~16度
#define LC_SET_TEMP_MAX  16  //冷藏设定温度最大值
#define LC_SET_TEMP_MIN  2   //冷藏设定温度最小值   


//冷藏压缩机连续运行保护  适用于定频压缩机，变频压缩机不能保护，一定要关掉保护功能
#define LC_COMP_RUN_MAX_1_HOUR   0  //冷藏压缩机连续运行最长时间1H,超过1小时，强制停机一段时间，以防温度过高损坏   1小时保护的优先级较高，有1小时保护，6小时就不需要保护了
#define LC_COMP_RUN_MAX_6_HOUR   0  //冷藏压缩机连续运行最长时间6H,超过6小时，强制停机一段时间，以防温度过高损坏


#define LD_COMP_RUN_MAX_1_HOUR   0  //冷冻压缩机连续运行最长时间1H,超过1小时，强制停机一段时间，以防温度过高损坏    1小时保护的优先级较高，有1小时保护，10小时就不需要保护了
#define LD_COMP_RUN_MAX_10_HOUR  0  //冷冻压缩机连续运行最长时间10H,超过10小时，强制停机一段时间，以防温度过高损坏


#endif
