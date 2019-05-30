#ifndef _TOOL_H_
#define _TOOL_H_

class Info
{
    public:
        double score;
        int ind_base;
        int ind_curr;
    public:
        Info();
        Info(double s, int ib, int ic);
};

struct Point
{
    double x;
    double y;
};


#endif
