#include <iostream>
#include <cmath>
#include <string.h>
#include "R2Graph.h"
#include <cassert>

using namespace std;

R2Point Nagelpoint (
    const R2Point& a, 
    const R2Point& b,
    const R2Point& c 
);

int main() {
    while(true) {
        R2Point a, b, c;
        cout << "Enter vertices of triangle:" << endl;
        if (!(cin >> a >> b >> c)) {
            break;
        }    
        R2Point npoint = Nagelpoint(a,b,c);
        cout << "NagelPoint is: " << npoint << endl;
        return 0;
    }
}
R2Point Nagelpoint (
    const R2Point& a, 
    const R2Point& b,
    const R2Point& c 
) {
    
    // вводим вектора и нормируем их

    R2Vector ab = b - a;
    ab.normalize();
    R2Vector ac = c - a;
    ac.normalize();
    R2Vector bc = c - b;
    bc.normalize();

    // ищем биссектрисы внешних углов

    R2Vector BAH = ab - ac;
    R2Vector ABH = (-1) * bc - ab;
    R2Vector CBH = bc + ab;
    R2Vector BCH = ac - bc;
    R2Vector ACH = bc - ac;
    R2Vector CAH = ac - ab;

    // будем искать центры вневписанных окружностей
    // первая всегда буква О
    // а далее  буквы соответствуют стороне, которой она касается 

    R2Point OAB;
    intersectStraightLines(
        a, BAH,
        b, ABH,
        OAB
    );
    R2Point OBC;
    intersectStraightLines(
        b, CBH,
        c, BCH,
        OBC
    );

    // будем прописывать векторы нормали
    // первая буква символизирует нормаль
    // а последующие - сторону, к которой ее провли

    R2Vector NAB = ab.normal();
    R2Vector NBC = bc.normal();

    R2Point Npoint;
    
    R2Point tpab; // touch points
    R2Point tpbc;
    
    intersectStraightLines(
        OAB, NAB,
        a, ab,
        tpab
    );

    intersectStraightLines(
        OBC, NBC,
        b, bc,
        tpbc
    );

    R2Vector ctpab = tpab - c; // vectors that unite touch poins
    R2Vector atpbc = tpbc - a; // with vertices

    intersectStraightLines (
        c, ctpab,
        a, atpbc,
        Npoint
    );

    return Npoint;
}
