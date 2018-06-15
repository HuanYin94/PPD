
/*** VORONOI.C ***/

#ifdef __cplusplus
extern "C" {
#endif

#include "vdefs.h"

extern Site * bottomsite ;
extern Halfedge * ELleftend, * ELrightend ;

/*** implicit parameters: nsites, sqrt_nsites, xmin, xmax, ymin, ymax,
 : deltax, deltay (can all be estimates).
 : Performance suffers if they are wrong; better to make nsites,
 : deltax, and deltay too big than too small.  (?)
 ***/

#define ADD_EDGE(e, edges) \
{\
    if (e->ep[LEFT_EDGE] != (Site *)NULL && e->ep[RIGHT_EDGE] != (Site *)NULL)\
    {\
        edges_s[*edges] = e->ep[LEFT_EDGE]->sitenbr;\
        edges_t[*edges] = e->ep[RIGHT_EDGE]->sitenbr;\
        (*edges)++;\
    }\
}

void
voronoi(Site *(*nextsite)(void), double vertexs_x[], double vertexs_y[], int edges_s[], int edges_t[], int *vertexs, int *edges)
    {
    Site * newsite, * bot, * top, * temp, * p, * v ;
    Point newintstar ;
    int pm ;
    Halfedge * lbnd, * rbnd, * llbnd, * rrbnd, * bisector ;
    Edge * e ;

    *vertexs = *edges = 0;

    PQinitialize() ;
    bottomsite = (*nextsite)() ;
    out_site(bottomsite) ;
    ELinitialize() ;
    newsite = (*nextsite)() ;
    while (1)
        {
        if(!PQempty())
            {
            newintstar = PQ_min() ;
            }
        if (newsite != (Site *)NULL && (PQempty()
            || newsite -> coord.y < newintstar.y
            || (newsite->coord.y == newintstar.y
            && newsite->coord.x < newintstar.x))) {/* new site is
smallest */
            {
            out_site(newsite) ;
            }
        lbnd = ELleftbnd(&(newsite->coord)) ;
        rbnd = ELright(lbnd) ;
        bot = rightreg(lbnd) ;
        e = bisect(bot, newsite) ;
        bisector = HEcreate(e, LEFT_EDGE) ;
        ELinsert(lbnd, bisector) ;
        p = intersect(lbnd, bisector) ;
        if (p != (Site *)NULL)
            {
            PQdelete(lbnd) ;
            PQinsert(lbnd, p, dist(p,newsite)) ;
            }
        lbnd = bisector ;
        bisector = HEcreate(e, RIGHT_EDGE) ;
        ELinsert(lbnd, bisector) ;
        p = intersect(bisector, rbnd) ;
        if (p != (Site *)NULL)
            {
            PQinsert(bisector, p, dist(p,newsite)) ;
            }
        newsite = (*nextsite)() ;
        }
    else if (!PQempty())   /* intersection is smallest */
            {
            lbnd = PQextractmin() ;
            llbnd = ELleft(lbnd) ;
            rbnd = ELright(lbnd) ;
            rrbnd = ELright(rbnd) ;
            bot = leftreg(lbnd) ;
            top = rightreg(rbnd) ;
            out_triple(bot, top, rightreg(lbnd)) ;
            v = lbnd->vertex ;
            makevertex(v) ;
            vertexs_x[*vertexs] = v->coord.x;
            vertexs_y[*vertexs] = v->coord.y;
            (*vertexs)++;
            endpoint(lbnd->ELedge, lbnd->ELpm, v);
            ADD_EDGE(lbnd->ELedge, edges);
            endpoint(rbnd->ELedge, rbnd->ELpm, v) ;
            ADD_EDGE(rbnd->ELedge, edges);
            ELdelete(lbnd) ;
            PQdelete(rbnd) ;
            ELdelete(rbnd) ;
            pm = LEFT_EDGE ;
            if (bot->coord.y > top->coord.y)
                {
                temp = bot ;
                bot = top ;
                top = temp ;
                pm = RIGHT_EDGE ;
                }
            e = bisect(bot, top) ;
            bisector = HEcreate(e, pm) ;
            ELinsert(llbnd, bisector) ;
            endpoint(e, RIGHT_EDGE-pm, v) ;
            ADD_EDGE(e, edges);
            deref(v) ;
            p = intersect(llbnd, bisector) ;
            if (p  != (Site *) NULL)
                {
                PQdelete(llbnd) ;
                PQinsert(llbnd, p, dist(p,bot)) ;
                }
            p = intersect(bisector, rrbnd) ;
            if (p != (Site *) NULL)
                {
                PQinsert(bisector, p, dist(p,bot)) ;
                }
            }
        else
            {
            break ;
            }
        }

    for( lbnd = ELright(ELleftend) ;
         lbnd != ELrightend ;
         lbnd = ELright(lbnd))
        {
        e = lbnd->ELedge ;
        out_ep(e) ;
        ADD_EDGE(e, edges);
        }
    }

#ifdef __cplusplus
} // extern "C"
#endif
