#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "Tools.h"
#include "RelativeMultiFlock.h"


//FILE *fr1, *fr2, *fr3, *fr4, *fr5;
FILE* fr;
int timestamp1[5], timestamp2[5], robotNum[2],bearing[2], orientation[2], range[2], tv[2], rv[2];
float xPos[5], yPos[5], heading[5];
int RCCstamp[5] = {0,0,0,0,0};
float tempX[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float tempY[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
int locked[5] = {0,0,0,0,0};
int ni = 1;
int idcnt[NUM_ROBOT];
inputdata input[548];
particleCloud E[NUM_ROBOT];
int theCounter = 0;
float guessX[NUM_ROBOT];
float guessY[NUM_ROBOT];


struct AprilTag{
    int RCCTimeStamp;
    float x;
    float y;
    float heading;
    int ATTimeStamp;
};

struct RCC{
    int RCCTimeStamp;
    int neighbor;
    int nBearing;
    int nOrientation;
    int nRange;
    int nTV;
    int nRV;
    float x;
    float y;
    float heading;
    int ATTimeStamp;
};

struct coordinates{
    float x;
    float y;
};

struct AprilTag Num95[2000];
struct RCC Num97[700];
struct AprilTag Num104[2000];
struct RCC Num110[700];
struct AprilTag Num113[2000];

int counter[5] = {0, 0, 0, 0, 0};

struct coordinates results[5][500];

int rcounter[5] = {0, 0, 0, 0, 0};

void changeSize(int w, int h) {
    
    // Prevent a divide by zero, when window is too short
    // (you cant make a window of zero width).
    if (h == 0)
        h = 1;
    
    float ratio =  w * 1.0 / h;
    
    // Use the Projection Matrix
    glMatrixMode(GL_PROJECTION);
    
    // Reset Matrix
    glLoadIdentity();
    
    // Set the viewport to be the entire window
    glViewport(0, 0, w, h);
    
    // Set the correct perspective.
    gluPerspective(45.0f, ratio, 0.1f, 100.0f);
    
    // Get Back to the Modelview
    glMatrixMode(GL_MODELVIEW);
}

void DrawCircle(float cx, float cy, float r, int num_segments)
{
    //glTranslatef(deltaX, 1.0f, 0.0f);
    glBegin(GL_LINE_LOOP);

    for(int ii = 0; ii < num_segments; ii++)
    {
        float theta = 2.0f * 3.1415926f * ii / num_segments;//get the current angle
        
        float x = r * cosf(theta);//calculate the x component
        float y = r * sinf(theta);//calculate the y component
        
        glVertex2f(x + cx, y + cy);//output vertex
        
    } 
    glEnd(); 
}


int ifIn(int* id, int size, int toFind){
    for (int i = 0; i < size; i++){
        if (id[i] == toFind) return 1;
    }
    return 0;
}

void relativerandcircinit(particleCloud* X, float bearing){
    float temp1[M];
    float temp2[M];
    for (int i = 0; i < M; i++){
        temp1[i] = (float)rand() / RAND_MAX;
        temp1[i] = (float)(6 * M_PI / 32) * (0.5 - temp1[i]) + bearing;
        
        temp2[i] = (float)rand() / RAND_MAX;
        temp2[i] = (1 - dead) * sqrt(temp2[i]) + dead;
        
        X -> partHeading[i] = M * rand() / RAND_MAX;
        X -> partHeading[i] = (0.5 - X -> partHeading[i]) * (M_PI / 32.0);
    }
    normalizeAngle(temp1);
    normalizeAngle(&(X -> partHeading[0]));
    
    cartCoordinates tempCart;
    cartCoordinates* toTempCart = &tempCart;
    pol2cart(toTempCart, temp1, temp2);
    for (int i = 0; i < M; i++){
        X -> partX[i] = tempCart.x[i];
        X -> partY[i] = tempCart.y[i];
    }
}

void readFile(){
    int c;
    int counter = 0;
    fr = fopen("/Users/chenxiaoyu/Documents/development/Test/Test/97AT2.log", "r");
    while ((c = fgetc(fr)) != EOF){
        ungetc(c, fr);
        fscanf(fr, "%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%d", &(input[counter].RCCTimeStamp), &(input[counter].neighbor), &(input[counter].nBearing), &(input[counter].nOrientation), &(input[counter].nRange), &(input[counter].nTV), &(input[counter].nRV), &(input[counter].x), &(input[counter].y), &(input[counter].heading), &(input[counter].ATTimeStamp));
        input[counter].nBearing /= 1000.0;
        input[counter].nOrientation /= 1000.0;
        input[counter].nRange /= 1000.0;
        input[counter].nTV /= 1000.0;
        input[counter].nRV /= 1000.0;
        input[counter].x /= 1000.0;
        input[counter].y /= 1000.0;
        input[counter].heading /= 1000.0;
        counter++;
    }
//    fr1 = fopen("/Users/chenxiaoyu/Documents/development/Test/Test/AT1.log", "r");
//    while ((c = fgetc(fr1)) != EOF){
//        ungetc(c, fr1);
//        struct AprilTag newLine;
//        fscanf(fr1, "%d,%f,%f,%f,%d", &(newLine.RCCTimeStamp), &(newLine.x), &(newLine.y), &(newLine.heading), &(newLine.ATTimeStamp));
//        if (rcounter[0] == 0 || newLine.RCCTimeStamp != Num95[counter[0] - 1].RCCTimeStamp){
//            struct coordinates newCor = {newLine.x, newLine.y};
//            results[0][rcounter[0]] = newCor;
//            rcounter[0] ++;
//        }
//        Num95[counter[0]] = newLine;
//        counter[0] ++;
//    }
//    fr2 = fopen("/Users/chenxiaoyu/Documents/development/Test/Test/97AT2.log", "r");
//    while ((c = fgetc(fr2)) != EOF){
//        ungetc(c, fr2);
//        struct RCC newLine;
//        fscanf(fr2, "%d,%d,%d,%d,%d,%d,%d,%f,%f,%f,%d", &(newLine.RCCTimeStamp), &(newLine.neighbor), &(newLine.nBearing), &(newLine.nOrientation), &(newLine.nRange), &(newLine.nTV), &(newLine.nRV), &(newLine.x), &(newLine.y), &(newLine.heading), &(newLine.ATTimeStamp));
//        if (rcounter[1] == 0 || newLine.RCCTimeStamp != Num97[counter[1] - 1].RCCTimeStamp){
//            struct coordinates newCor = {newLine.x, newLine.y};
//            results[1][rcounter[1]] = newCor;
//            rcounter[1] ++;
//        }
//        Num97[counter[1]] = newLine;
//        counter[1] ++;
//    }
//    fr3 = fopen("/Users/chenxiaoyu/Documents/development/Test/Test/AT3.log", "r");
//    while ((c = fgetc(fr3)) != EOF){
//        ungetc(c, fr3);
//        struct AprilTag newLine;
//        fscanf(fr3, "%d,%f,%f,%f,%d", &(newLine.RCCTimeStamp), &(newLine.x), &(newLine.y), &(newLine.heading), &(newLine.ATTimeStamp));
//        if (rcounter[2] == 0 || newLine.RCCTimeStamp != Num104[counter[2] - 1].RCCTimeStamp){
//            struct coordinates newCor = {newLine.x, newLine.y};
//            results[2][rcounter[2]] = newCor;
//            rcounter[2] ++;
//        }
//        Num104[counter[2]] = newLine;
//        counter[2] ++;
//    }
//    fr4 = fopen("/Users/chenxiaoyu/Documents/development/Test/Test/110AT4.log", "r");
//    while ((c = fgetc(fr4)) != EOF){
//        ungetc(c, fr4);
//        struct RCC newLine;
//        fscanf(fr4, "%d,%d,%d,%d,%d,%d,%d,%f,%f,%f,%d", &(newLine.RCCTimeStamp), &(newLine.neighbor), &(newLine.nBearing), &(newLine.nOrientation), &(newLine.nRange), &(newLine.nTV), &(newLine.nRV), &(newLine.x), &(newLine.y), &(newLine.heading), &(newLine.ATTimeStamp));
//        if (rcounter[3] == 0 || newLine.RCCTimeStamp != Num110[counter[3] - 1].RCCTimeStamp){
//            struct coordinates newCor = {newLine.x, newLine.y};
//            results[3][rcounter[3]] = newCor;
//            rcounter[3] ++;
//        }
//        Num110[counter[3]] = newLine;
//        counter[3] ++;
//    }
//    fr5 = fopen("/Users/chenxiaoyu/Documents/development/Test/Test/AT5.log", "r");
//    while ((c = fgetc(fr5)) != EOF){
//        ungetc(c, fr5);
//        struct AprilTag newLine;
//        fscanf(fr5, "%d,%f,%f,%f,%d", &(newLine.RCCTimeStamp), &(newLine.x), &(newLine.y), &(newLine.heading), &(newLine.ATTimeStamp));
//        if (rcounter[4] == 0 || newLine.RCCTimeStamp != Num113[counter[4] - 1].RCCTimeStamp){
//            struct coordinates newCor = {newLine.x, newLine.y};
//            results[4][rcounter[4]] = newCor;
//            rcounter[4] ++;
//        }
//        Num113[counter[4]] = newLine;
//        counter[4] ++;
//    }
}

int drawCounter[5] = {0, 0, 0, 0, 0};

void renderScene(void) {
    // Clear Color and Depth Buffers
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // Reset transformations
    glLoadIdentity();
    // Set the camera
    gluLookAt(	1.0f, 1.0f, 7.0f,
              1.0f, 1.0f,  0.0f,
              0.0f, 1.0f,  0.0f);

    if (ifIn(idcnt, NUM_ROBOT, input[ni].neighbor) == 0){
        relativerandcircinit(E + theCounter, input[ni].nBearing);
        idcnt[theCounter] = input[ni].neighbor;
        theCounter ++;
    }
    int current_index = -1;
    for (int j = 0; j < theCounter; j++){
        if (idcnt[j] == input[ni].neighbor){
            current_index = j;
            break;
        }
    }
    
    particleFilter(E + current_index, input[ni].nBearing, input[ni].nOrientation, input[ni].nRange, input[ni].nTV, input[ni].nRV);
    
    Guess newGuess;
    newGuess.x = 0;
    newGuess.y = 0;
    newGuess.heading = 0;
    Guess* toNewGuess = &newGuess;
    updateGuess(toNewGuess, E + current_index);
    guessX[current_index] = newGuess.x;
    guessY[current_index] = newGuess.y;
    
    for (int j = 0; j < M; j++){
        E[current_index].weight[j] = E[current_index].nextWeight[j];
    }
    
    ni++;
    
    for (int j = 0; j < theCounter; j++){
        glColor3f(1,1,1);
//        printf("%f, %f\n", guessX[j], guessY[j]);
        DrawCircle(guessX[j], guessY[j], 0.15, 100);
    }
    
    glColor3f(1,0,0);
    DrawCircle(0, 0, 0.15, 100);
//    if (drawCounter[0] < rcounter[0]){
//        drawCounter[0] ++;
//    }
//    
//    //yellow NUM 97
//    if (drawCounter[1] < rcounter[1]){
//        drawCounter[1] ++;
//    }
//    
//    //green NUM 104
//    if (drawCounter[2] < rcounter[2]){
//        drawCounter[2] ++;
//    }
//    
//    //blue NUM 110
//    if (drawCounter[3] < rcounter[3]){
//        drawCounter[3] ++;
//    }
//    
//    //purple NUM 113
//    if (drawCounter[4] < rcounter[4]){
//        drawCounter[4] ++;
//    }
//    
//    glColor3f(1.0, 0, 0);
//    DrawCircle(results[0][drawCounter[0] - 1].x / 100, results[0][drawCounter[0] - 1].y / 100, 0.15, 100);
//    
//    glColor3f(1.0, 1.0, 0);
//    DrawCircle(results[1][drawCounter[1] - 1].x / 100, results[1][drawCounter[1] - 1].y / 100, 0.15, 100);
//    
//    glColor3f(0, 1.0, 0);
//    DrawCircle(results[2][drawCounter[2] - 1].x / 100, results[2][drawCounter[2] - 1].y / 100, 0.15, 100);
//    
//    glColor3f(0, 0, 1.0);
//    DrawCircle(results[3][drawCounter[3] - 1].x / 100, results[3][drawCounter[3] - 1].y / 100, 0.15, 100);
//    
//    glColor3f(1.0, 0, 1.0);
//    DrawCircle(results[4][drawCounter[4] - 1].x / 100, results[4][drawCounter[4] - 1].y / 100, 0.15, 100);
    
    glutSwapBuffers();
}

int main(int argc, char **argv) {

    simulationInit(E);
    // init GLUT and create window
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowPosition(100,100);
    glutInitWindowSize(640,640);
    glutCreateWindow("Particle Filter Demo");
    
    readFile();
    // register callbacks
    glutDisplayFunc(renderScene);
    glutReshapeFunc(changeSize);
    glutIdleFunc(renderScene);

    
    // enter GLUT event processing cycle
    glutMainLoop();
    
    return 1;
}