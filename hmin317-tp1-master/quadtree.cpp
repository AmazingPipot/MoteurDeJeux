#include "quadtree.h"
#include <cstdlib>
#include <iostream>
#include <math.h>
/*QuadTree::QuadTree()
{

}*/

QuadTree::QuadTree() :
    profMax(5),
    X(0),
    Y(0),
    longueur(0),
    largeur(0),
    Nord(NULL),
    Est(NULL),
    Ouest(NULL),
    Sud(NULL),
    distMin(0),
    XX(0),
    YY(0)

{

}
QuadTree::QuadTree(float x, float y, float l, float L, float xx, float yy, float d, float dm, int p) :
    X(x),
    Y(y),
    XX(xx),
    YY(yy),
    distMin(d),
    distMax(dm),
    longueur(l),
    largeur(L),
    profondeur(p)
{
    lod();
}
QuadTree::QuadTree(float x, float y, float l, float L, int p) :
    X(x),
    Y(y),

    longueur(l),
    largeur(L),
    profondeur(p)
{
    initFils();
}

QuadTree::~QuadTree()
{

}

void QuadTree::lod()
{
    float dist;// = sqrt(pow(XX-X,2)+pow(YY-Y,2));
    int p;

    //std::cout<<"XX "<< X <<"YY "<< Y << std::endl;
    //std::cout<<"dist "<< dist << "DISTMIN" << distMin << std::endl;
    dist = sqrt(pow(XX-X,2)+pow(YY-Y,2));
    std::cout<<"dist "<< dist << "DISTMIN" << distMin << std::endl;

    if(profondeur >= 0){
        if (dist > distMax){
            Nord = NULL;
            Est = NULL;
            Sud = NULL;
            Ouest = NULL;
            return;
        }
        else if (longueur > distMin && 6*longueur > dist){
            Nord = new QuadTree(X,Y,longueur/2,largeur/2,XX,YY,distMin,distMax,profondeur-1);
            Est = new QuadTree(X+longueur/2,Y,longueur/2,largeur/2,XX,YY,distMin,distMax,profondeur-1);
            Sud = new QuadTree(X+longueur/2,Y+largeur/2,longueur/2,largeur/2,XX,YY,distMin,distMax,profondeur-1);
            Ouest = new QuadTree(X,Y+largeur/2,longueur/2,largeur/2,XX,YY,distMin,distMax,profondeur-1);
        }
        else {
            Nord = NULL;
            Est = NULL;
            Sud = NULL;
            Ouest = NULL;
            return;
        }
    }
    else {
        Nord = NULL;
        Est = NULL;
        Sud = NULL;
        Ouest = NULL;
        return;
    }
}
void QuadTree::initFils()
{
    //float dist = sqrt(pow(XX-X,2)+pow(YY-Y,2));

    //std::cout<<"dist "<< dist << std::endl;

    if (profondeur >= 0){
        Nord = new QuadTree(X,Y,longueur/2,largeur/2,profondeur-1);
        Est = new QuadTree(X+longueur/2,Y,longueur/2,largeur/2,profondeur-1);
        Sud = new QuadTree(X+longueur/2,Y+largeur/2,longueur/2,largeur/2,profondeur-1);
        Ouest = new QuadTree(X,Y+largeur/2,longueur/2,largeur/2,profondeur-1);
    }
    else{
        Nord = NULL;
        Est = NULL;
        Sud = NULL;
        Ouest = NULL;
    }
}

int QuadTree::feuille()
{
    if (!Nord)
    {
        return 1;
    }
    else
    {
        //return Nord->feuille();
        return Nord->feuille()+Sud->feuille()+Est->feuille()+Ouest->feuille();
    }
}
