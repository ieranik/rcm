#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <algorithm>
#include <time.h>
#include <stdlib.h>
#include <map>
#include <conio.h>
#include <unordered_set>

#include <windows.h>
#include <glut.h>

using namespace std;

#define pi (2*acos(0.0))
#define eps 0.000001

#define N 120
#define cpx 4
#define numobs 60
#define rrange (N * cpx)
#define crange 60
#define osa 8
#define osd 5
#define numtarget 10
#define maxcol 128

int G[N][N];
bool B[N][N];
int T;

FILE* fout;

vector< vector<int> > AL;




double max2(double a,double b)
{
    return (a>b)?a:b;
}

double min2(double a,double b)
{
    return (a<b)?a:b;
}


class point
{
public:
    double x,y;
    point(double xx=0.0, double yy=0.0)
    {
        x=xx;
        y=yy;
    }
};

class cell
{
public:
    int x, y;
    point c;
    cell(int xx = 0, int yy = 0)
    {
        x = xx;
        y = yy;
        c.x = (x + 0.5) * cpx;
        c.y = (y + 0.5) * cpx;
    }
};

class obs
{
public:
    double xl,xh,yl,yh;
    obs(double xxl=0.0, double xxh=0.0, double yyl=0.0, double yyh=0.0)
    {
        xl=xxl;
        xh=xxh;
        yl=yyl;
        yh=yyh;
    }
};


class robot
{
public:
    int ind;
    int cnt;
    robot(int i = -1, int c = 0)
    {
        ind = i;
        cnt = c;
    }
};

bool comparator(const robot& lhs, const robot& rhs)
{
   return lhs.cnt < rhs.cnt;
}

bool rcomparator(const robot& lhs, const robot& rhs)
{
   return lhs.cnt > rhs.cnt;
}


vector<obs> obsdata;
vector<cell> targetdata;
vector<int> targetdir = vector<int>(numtarget, 0);


void po(obs o)
{
    cout<<o.xl<<" "<<o.xh<<" "<<o.yl<<" "<<o.yh<<endl;
}

bool doesint1(obs a,obs b)
{
    if(((b.xl <= a.xl && a.xl <= b.xh)||(b.xl <= a.xh && a.xh <= b.xh)||(a.xl <= b.xl && b.xl <= a.xh)||(a.xl <= b.xh && b.xh <= a.xh))&&((b.yl <= a.yl && a.yl <= b.yh)||(b.yl <= a.yh && a.yh <= b.yh)||(a.yl <= b.yl && b.yl <= a.yh)||(a.yl <= b.yh && b.yh <= a.yh)))return true;

    return false;
}

bool doesintn(vector<obs> ol,obs o)
{
    int i;
    for(i=0;i<ol.size();i++)if(doesint1(ol[i],o)==true)return true;

    return false;
}

bool isinside1(obs o, point p)
{
    if(o.xl<p.x+eps&&p.x<o.xh+eps&&o.yl<p.y+eps&&p.y<o.yh+eps)return true;

    return false;
}

bool isinsiden(vector<obs> ol, point p)
{
    int i;
    for(i=0;i<ol.size();i++)if(isinside1(ol[i],p)==true)return true;

    return false;
}




void genobs()
{
    obsdata.clear();

    obs o;
    int i,ri,rii;
    double rd;
    srand(time(NULL));
    int cnt=numobs;
    while(cnt!=0)
    {
        o.xl=rand()%(N-osa-osd);
        ri=rand()%(2*osd)+(osa-osd);
        o.xh=o.xl+ri;

        o.yl=rand()%(N-osa-osd);
        ri=rand()%(2*osd)+(osa-osd);
        o.yh=o.yl+ri;

        o.xl *= cpx;
        o.xh *= cpx;
        o.yl *= cpx;
        o.yh *= cpx;


        if(doesintn(obsdata,o)==false)
        {
            obsdata.push_back(o);
            cnt--;
        }
    }
}

void genblockedcells()
{
    int i, j, k;

	for (i = 0; i < N; i++)
    {
        for (j = 0; j < N; j++)
        {
            point tp;
            tp.x = i * cpx + (cpx / 2.0);
            tp.y = j * cpx + (cpx / 2.0);

            bool flag = true;
            for (k = 0; k < numobs; k++)
            {
                if (isinside1(obsdata[k], tp) == true)
                {
                    flag = false;
                    break;
                }
            }
            B[i][j] = flag;
        }
    }
}

void gentarget()
{
    targetdata.clear();

    int cnt = numtarget;
    while(cnt != 0)
    {
        int cx = rand() % N;
        int cy = rand() % N;

        if (B[cx][cy] == true)
        {
            targetdata.push_back(cell(cx, cy));
            cnt--;
        }

    }
}




double area3p(point a, point b, point c)
{
    return a.x*(b.y-c.y)+b.x*(c.y-a.y)+c.x*(a.y-b.y);
}

bool doesint(point a, point b, point c, point d)
{
    if(area3p(a,b,c)*area3p(a,b,d)<=0&&area3p(c,d,a)*area3p(c,d,b)<=0)
        return true;
    return false;
}

vector<obs> rangequery(point p)
{
    int j;
    vector<obs> ret;
    obs to;
    to.xl=p.x-crange;
    to.xh=p.x+crange;
    to.yl=p.y-crange;
    to.yh=p.y+crange;
    for(j=0;j<obsdata.size();j++)
    {
        if(doesint1(obsdata[j],to)==true)ret.push_back(obsdata[j]);
    }
    return ret;
}




void drawSquare(double x, double y, double s)
{
	glBegin(GL_QUADS);{
		glVertex3f( x + s, y + s, 0);
		glVertex3f( x + s, y - s, 0);
		glVertex3f( x - s, y - s, 0);
		glVertex3f( x - s, y + s, 0);
	}glEnd();
}

void drawCircle(double x, double y, double radius)
{
    int i;
    int segments = 16;
    struct point points[17];
    //generate points
    for(i=0;i<=segments;i++)
    {
        points[i].x = x + radius*cos(((double)i/(double)segments)*2*pi);
        points[i].y = y + radius*sin(((double)i/(double)segments)*2*pi);
    }
    //draw segments using generated points
    for(i=0;i<segments;i++)
    {
        glBegin(GL_TRIANGLES);
        {
            glVertex3f(x, y, 0);
			glVertex3f(points[i].x,points[i].y,0);
			glVertex3f(points[i+1].x,points[i+1].y,0);
        }
        glEnd();
    }
}




void keyboardListener(unsigned char key, int x,int y){
	switch(key){

		case '1':
			//drawgrid = 1 - drawgrid;
			break;

		default:
			break;
	}
}


void specialKeyListener(int key, int x,int y){
	switch(key){
		case GLUT_KEY_DOWN:		//down arrow key
			//cameraHeight -= 3.0;
			break;
		case GLUT_KEY_UP:		// up arrow key
			//cameraHeight += 3.0;
			break;

		case GLUT_KEY_RIGHT:
			//cameraAngle += 0.03;
			break;
		case GLUT_KEY_LEFT:
			//cameraAngle -= 0.03;
			break;

		case GLUT_KEY_PAGE_UP:
			break;
		case GLUT_KEY_PAGE_DOWN:
			break;

		case GLUT_KEY_INSERT:
			break;

		case GLUT_KEY_HOME:
			break;
		case GLUT_KEY_END:
			break;

		default:
			break;
	}
}


void mouseListener(int button, int state, int x, int y){	//x, y is the x-y of the screen (2D)
	switch(button){
		case GLUT_LEFT_BUTTON:
			if(state == GLUT_DOWN){		// 2 times?? in ONE click? -- solution is checking DOWN or UP

			}
			break;

		case GLUT_RIGHT_BUTTON:
			//........
			break;

		case GLUT_MIDDLE_BUTTON:
			//........
			break;

		default:
			break;
	}
}





void display(){

	//clear the display
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0,0,0,0);	//color black
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	/********************
	/ set-up camera here
	********************/
	//load the correct matrix -- MODEL-VIEW matrix
	glMatrixMode(GL_MODELVIEW);

	//initialize the matrix
	glLoadIdentity();

	//now give three info
	//1. where is the camera (viewer)?
	//2. where is the camera looking?
	//3. Which direction is the camera's UP direction?

	//gluLookAt(100,100,100,	0,0,0,	0,0,1);
	//gluLookAt(200*cos(cameraAngle), 200*sin(cameraAngle), cameraHeight,		0,0,0,		0,0,1);
	//gluLookAt(0,0,200,	0,0,0,	0,1,0);
	gluLookAt(0,0,0,	0,0,-1,	0,1,0);


	//again select MODEL-VIEW
	glMatrixMode(GL_MODELVIEW);


	/****************************
	/ Add your objects from here
	****************************/
	//add objects


    int i,j;


    glColor3f(1,0,0);

    for(i=0;i<obsdata.size();i++)
    {
        glBegin(GL_QUADS);
        {
			glVertex3f(obsdata[i].xl,obsdata[i].yl,0);
			glVertex3f(obsdata[i].xh,obsdata[i].yl,0);
			glVertex3f(obsdata[i].xh,obsdata[i].yh,0);
			glVertex3f(obsdata[i].xl,obsdata[i].yh,0);
        }
        glEnd();
    }


    glColor3f(0,0,1);
    for (i = 0; i < targetdata.size(); i++) drawCircle(targetdata[i].c.x, targetdata[i].c.y, 4.0);



    for (i = 0; i < N; i++)
    {
        for (j = 0; j < N; j++)
        {
            glColor3f(0, G[i][j] / 100.0, 0);
            glBegin(GL_QUADS);
            {
                glVertex3f(i * cpx, j * cpx, 0);
                glVertex3f((i + 1) * cpx, j * cpx, 0);
                glVertex3f((i + 1) * cpx, (j + 1) * cpx, 0);
                glVertex3f(i * cpx, (j + 1) * cpx, 0);
            }
            glEnd();
        }
    }

	//ADD this line in the end --- if you use double buffer (i.e. GL_DOUBLE)
	glutSwapBuffers();
}

void animate(int val){
	//codes for any changes in Models, Camera


    int i, j, k, l;
	if (T % 10 == 0)
    {
        vector<robot> RC;
        for (i = 0; i < targetdata.size(); i++)
        {
            int cnt, gx, gy;
            unordered_set<int> st;
            st.clear();

            gx = targetdata[i].x;
            gy = targetdata[i].y;
            cnt = 10;
            while (++gy < N && cnt-- && B[gx][gy] == true)
            {
                for (j = 0; j < AL[gx * N + gy].size(); j++) st.insert(AL[gx * N + gy][j]);
            }

            gx = targetdata[i].x;
            gy = targetdata[i].y;
            cnt = 10;
            while (--gy >= 0 && cnt-- && B[gx][gy] == true)
            {
                for (j = 0; j < AL[gx * N + gy].size(); j++) st.insert(AL[gx * N + gy][j]);
            }

            gx = targetdata[i].x;
            gy = targetdata[i].y;
            cnt = 10;
            while (++gx < N && cnt-- && B[gx][gy] == true)
            {
                for (j = 0; j < AL[gx * N + gy].size(); j++) st.insert(AL[gx * N + gy][j]);
            }

            gx = targetdata[i].x;
            gy = targetdata[i].y;
            cnt = 10;
            while (--gx >= 0 && cnt-- && B[gx][gy] == true)
            {
                for (j = 0; j < AL[gx * N + gy].size(); j++) st.insert(AL[gx * N + gy][j]);
            }

            robot rb;
            rb.cnt = 0;
            rb.ind = i;

            for (auto it = st.begin(); it != st.end(); it++) rb.cnt += 100 - G[*it / N][*it % N];

            RC.push_back(rb);
        }


        sort(RC.begin(), RC.end(), &comparator);
//        for (i = 0; i < RC.size(); i++)
//            cout << targetdata[RC[i].ind].x << " " << targetdata[RC[i].ind].y << " " << RC[i].cnt << endl;



        int GC[N][N];
        for (i = 0; i < N; i++) for (j = 0; j < N; j++) GC[i][j] = G[i][j];


        for (i = 0; i < RC.size(); i++)
        {

            int ind = RC[i].ind;


            int cnt, gx, gy;
            unordered_set<int> st;



            st.clear();
            gx = targetdata[ind].x;
            gy = targetdata[ind].y;
            cnt = 10;
            while (++gy < N && cnt-- && B[gx][gy] == true)
            {
                for (j = 0; j < AL[gx * N + gy].size(); j++) st.insert(AL[gx * N + gy][j]);
            }

            int tt = 0;
            for (auto it = st.begin(); it != st.end(); it++) tt += 100 - GC[*it / N][*it % N];



            st.clear();
            gx = targetdata[ind].x;
            gy = targetdata[ind].y;
            cnt = 10;
            while (--gy >=0 && cnt-- && B[gx][gy] == true)
            {
                for (j = 0; j < AL[gx * N + gy].size(); j++) st.insert(AL[gx * N + gy][j]);
            }

            int bb = 0;
            for (auto it = st.begin(); it != st.end(); it++) bb += 100 - GC[*it / N][*it % N];



            st.clear();
            gx = targetdata[ind].x;
            gy = targetdata[ind].y;
            cnt = 10;
            while (++gx < N && cnt-- && B[gx][gy] == true)
            {
                for (j = 0; j < AL[gx * N + gy].size(); j++) st.insert(AL[gx * N + gy][j]);
            }

            int rr = 0;
            for (auto it = st.begin(); it != st.end(); it++) rr += 100 - GC[*it / N][*it % N];



            st.clear();
            gx = targetdata[ind].x;
            gy = targetdata[ind].y;
            cnt = 10;
            while (--gx >=0 && cnt-- && B[gx][gy] == true)
            {
                for (j = 0; j < AL[gx * N + gy].size(); j++) st.insert(AL[gx * N + gy][j]);
            }

            int ll = 0;
            for (auto it = st.begin(); it != st.end(); it++) ll += 100 - GC[*it / N][*it % N];


//            cout << tt << endl;
//            cout << bb << endl;
//            cout << rr << endl;
//            cout << ll << endl;
//
//            cout << ind << endl;


            if (tt >= bb && tt >= ll && tt >= rr)
            {
                //cout << "t" << endl;
                targetdir[ind] = 1;
                st.clear();
                gx = targetdata[ind].x;
                gy = targetdata[ind].y;
                cnt = 10;
                while (++gy < N && cnt-- && B[gx][gy] == true)
                {
                    for (j = 0; j < AL[gx * N + gy].size(); j++) st.insert(AL[gx * N + gy][j]);
                }
                for (auto it = st.begin(); it != st.end(); it++) GC[*it / N][*it % N] = 100;

            }
            else if (bb >= tt && bb >= ll && bb >= rr)
            {
                //cout << "b" << endl;
                targetdir[ind] = 2;
                st.clear();
                gx = targetdata[ind].x;
                gy = targetdata[ind].y;
                cnt = 10;
                while (--gy >= 0 && cnt-- && B[gx][gy] == true)
                {
                    for (j = 0; j < AL[gx * N + gy].size(); j++) st.insert(AL[gx * N + gy][j]);
                }
                for (auto it = st.begin(); it != st.end(); it++) GC[*it / N][*it % N] = 100;

            }
            else if (rr >= tt && rr >= ll && rr >= bb)
            {
                //cout << "r" << endl;
                targetdir[ind] = 3;
                st.clear();
                gx = targetdata[ind].x;
                gy = targetdata[ind].y;
                cnt = 10;
                while (++gx < N && cnt-- && B[gx][gy] == true)
                {
                    for (j = 0; j < AL[gx * N + gy].size(); j++) st.insert(AL[gx * N + gy][j]);
                }
                for (auto it = st.begin(); it != st.end(); it++) GC[*it / N][*it % N] = 100;

            }
            else
            {
                //cout << "l" << endl;
                targetdir[ind] = 4;
                st.clear();
                gx = targetdata[ind].x;
                gy = targetdata[ind].y;
                cnt = 10;
                while (--gx >= 0 && cnt-- && B[gx][gy] == true)
                {
                    for (j = 0; j < AL[gx * N + gy].size(); j++) st.insert(AL[gx * N + gy][j]);
                }
                for (auto it = st.begin(); it != st.end(); it++) GC[*it / N][*it % N] = 100;

            }

        }

    }

    for (i = 0; i < targetdir.size(); i++) cout << targetdir[i] << " ";
    cout << endl;

//    for (i = 0; i < targetdata.size(); i++)
//        cout << targetdata[i].x << " " << targetdata[i].y << " " << targetdir[i] << endl;


    unordered_set<int> st;


    for (i = 0; i < targetdata.size(); i++)
    {
        if (targetdir[i] == 1)
        {
            if (targetdata[i].y < N - 1 && B[targetdata[i].x][targetdata[i].y + 1] == true)
                targetdata[i] = cell(targetdata[i].x, targetdata[i].y + 1);
        }
        else if (targetdir[i] == 2)
        {
            if (targetdata[i].y > 0 && B[targetdata[i].x][targetdata[i].y - 1] == true)
                targetdata[i] = cell(targetdata[i].x, targetdata[i].y - 1);
        }
        else if (targetdir[i] == 3)
        {
            if (targetdata[i].x < N - 1 && B[targetdata[i].x + 1][targetdata[i].y] == true)
                targetdata[i] = cell(targetdata[i].x + 1, targetdata[i].y);
        }
        else if (targetdir[i] == 4)
        {
            if (targetdata[i].x > 0 && B[targetdata[i].x - 1][targetdata[i].y] == true)
                targetdata[i] = cell(targetdata[i].x - 1, targetdata[i].y);
        }

        for (j = 0; j < AL[targetdata[i].x * N + targetdata[i].y].size(); j++)
                st.insert(AL[targetdata[i].x * N + targetdata[i].y][j]);
    }

    for (i = 0; i < N * N; i++)
    {
        auto it = st.find(i);
        if (it != st.end()) G[i / N][i % N] = 100;
        else if (G[i / N][i % N] > 0) G[i / N][i % N]--;
    }

    T++;

    glutPostRedisplay();
	glutTimerFunc(50, animate, 0);
}


//calculate visibility of cells around a robot
void processtarget(cell c)
{
    point p = c.c;
    vector<obs> vo = rangequery(p);
    int lgn = max(0, c.x - (int)(crange / cpx));
    int rgn = min(N - 1, c.x + (int)(crange / cpx));
    int bgn = max(0, c.y - (int)(crange / cpx));
    int tgn = min(N - 1, c.y + (int)(crange / cpx));

    int i, j, k;
    for (i = lgn; i <= rgn; i++)
    {
        for (j = bgn; j <= tgn; j++)
        {
            point tp = point((i + 0.5) * cpx, (j + 0.5) * cpx);
            if (B[i][j] == false) continue;
            if ((p.x - tp.x) * (p.x - tp.x) + (p.y - tp.y) * (p.y - tp.y) > crange * crange) continue;
            bool flag = true;
            for (k = 0; k < vo.size(); k++)
            {
                if (doesint(p, tp, point(vo[k].xl, vo[k].yl), point(vo[k].xl, vo[k].yh)))
                {
                    flag = false;
                    break;
                }
                if (doesint(p, tp, point(vo[k].xh, vo[k].yl), point(vo[k].xh, vo[k].yh)))
                {
                    flag = false;
                    break;
                }
                if (doesint(p, tp, point(vo[k].xl, vo[k].yl), point(vo[k].xh, vo[k].yl)))
                {
                    flag = false;
                    break;
                }
                if (doesint(p, tp, point(vo[k].xl, vo[k].yh), point(vo[k].xh, vo[k].yh)))
                {
                    flag = false;
                    break;
                }
            }
            if (flag == true)
            {
                G[i][j] = 100;
            }
        }
    }
}

//find the cells visible from a given cell
vector<int> processcell(cell c)
{
    point p = c.c;
    vector<int> ret;
    ret.clear();
    vector<obs> vo = rangequery(p);
    int lgn = max(0, c.x - (int)(crange / cpx));
    int rgn = min(N - 1, c.x + (int)(crange / cpx));
    int bgn = max(0, c.y - (int)(crange / cpx));
    int tgn = min(N - 1, c.y + (int)(crange / cpx));

    int i, j, k;
    for (i = lgn; i <= rgn; i++)
    {
        for (j = bgn; j <= tgn; j++)
        {
            point tp = point((i + 0.5) * cpx, (j + 0.5) * cpx);
            if (B[i][j] == false) continue;
            if ((p.x - tp.x) * (p.x - tp.x) + (p.y - tp.y) * (p.y - tp.y) > crange * crange) continue;
            bool flag = true;
            for (k = 0; k < vo.size(); k++)
            {
                if (doesint(p, tp, point(vo[k].xl, vo[k].yl), point(vo[k].xl, vo[k].yh)))
                {
                    flag = false;
                    break;
                }
                if (doesint(p, tp, point(vo[k].xh, vo[k].yl), point(vo[k].xh, vo[k].yh)))
                {
                    flag = false;
                    break;
                }
                if (doesint(p, tp, point(vo[k].xl, vo[k].yl), point(vo[k].xh, vo[k].yl)))
                {
                    flag = false;
                    break;
                }
                if (doesint(p, tp, point(vo[k].xl, vo[k].yh), point(vo[k].xh, vo[k].yh)))
                {
                    flag = false;
                    break;
                }
            }
            if (flag == true)
            {
                ret.push_back(i * N + j);
            }
        }
    }
    return ret;
}


void init(){
	//codes for initialization

	int i, j, k;
    T = 0;

	genobs();
	genblockedcells();
    gentarget();



    fout = fopen("obs.txt", "w");
    fprintf(fout, "%d\n", numobs);
    for (i = 0; i < numobs; i++) fprintf(fout,"%f %f %f %f\n", obsdata[i].xl, obsdata[i].xh, obsdata[i].yl, obsdata[i].yh);
    fclose(fout);

//    fprintf(fout, "%d\n", numtarget);
//    for (i = 0; i < numtarget; i++)
//        fprintf(fout,"%lf %lf %lf %lf\n", targetdata[i].xl, obsdata[i].xh, obsdata[i].yl, obsdata[i].yh);


    for (i = 0; i < N; i++) for (j = 0; j < N; j++) G[i][j] = 0;

    for (i = 0; i < targetdata.size(); i++) processtarget(targetdata[i]);

    AL.clear();
    for (i = 0; i < N; i++) for (j = 0; j < N; j++) AL.push_back(processcell(cell(i, j)));







	//clear the screen
	glClearColor(0,0,0,0);

	/************************
	/ set-up projection here
	************************/
	//load the PROJECTION matrix
	glMatrixMode(GL_PROJECTION);

	//initialize the matrix
	glLoadIdentity();

	//give PERSPECTIVE parameters
	gluOrtho2D(0, N * cpx, 0, N * cpx);
	//gluPerspective(80,	1,	1,	1000.0);
	//field of view in the Y (vertically)
	//aspect ratio that determines the field of view in the X direction (horizontally)
	//near distance
	//far distance
}

int main(int argc, char **argv){
	glutInit(&argc,argv);
	glutInitWindowSize(N * cpx, N * cpx);
	glutInitWindowPosition(0, 0);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGB);	//Depth, Double buffer, RGB color

	glutCreateWindow("My OpenGL Program");

	init();

	glEnable(GL_DEPTH_TEST);	//enable Depth Testing

	glutDisplayFunc(display);	//display callback function
	glutTimerFunc(50, animate, 0);		//what you want to do in the idle time (when no drawing is occuring)

	//glutIdleFunc(animate);

	glutKeyboardFunc(keyboardListener);
	glutSpecialFunc(specialKeyListener);
	glutMouseFunc(mouseListener);

	glutMainLoop();		//The main loop of OpenGL

	return 0;
}


