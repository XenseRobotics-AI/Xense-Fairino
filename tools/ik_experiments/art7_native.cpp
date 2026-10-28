// Original offline ART7 numerical core. C++17, standard library only.
// No robot interface, network calls, third-party IK code, or global state.
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace {
constexpr int N = 7;
constexpr double PI = 3.141592653589793238462643383279502884;
constexpr double DEG = 180.0 / PI;
constexpr double RS = 200.0;
constexpr double XS[] = {0,1,2,4,5,6,8,10,12,14,16,18,20,22,24,26,28,30,32,34,36,38,40,42,44,46,48,50,52,54,56,58,60,62,64,66,68,70};
constexpr double YS[] = {85,85,85,80.5,77.6,73.7,66.3,61.4,58,55.5,53.5,51.9,50.6,49.5,48.5,47.6,46.8,46.1,45.5,44.9,44.3,43.7,43.2,42.7,42.1,41.6,41.2,40.7,40.2,39.7,39.2,38.6,38.1,37.6,37.1,36.5,35.9,35.3};
constexpr int NB = sizeof(XS)/sizeof(XS[0]);
using V = double[N];
using H = double[N][N];

double clamp(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }
double norm3(const double *v) { return std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]); }
void copy7(const double *a, double *b) { std::copy(a, a+N, b); }
bool finite(const double *p, int n) { for(int i=0;i<n;i++) if(!std::isfinite(p[i])) return false; return true; }

void fk(const double *q, const double *offset, double *out, double jac[6][7]) {
    double r[3][3]={{1,0,0},{0,1,0},{0,0,1}}, p[3]={0,0,0};
    double origins[7][3], axes[7][3];
    constexpr double d[7]={187,0,290,0,310,0,0};
    for(int k=0;k<7;k++) {
        for(int i=0;i<3;i++) { origins[k][i]=p[i]; axes[k][i]=r[i][2]; }
        double t=q[k]+(k==5 ? PI/2 : 0.0), c=std::cos(t), s=std::sin(t);
        double alpha=k==6 ? 0.0 : (k%2==0 ? -PI/2 : PI/2);
        double ca=std::cos(alpha), sa=std::sin(alpha);
        double b[3][3]={{c,-s*ca,s*sa},{s,c*ca,-c*sa},{0,sa,ca}}, next[3][3]={};
        for(int i=0;i<3;i++) {
            p[i]+=r[i][2]*d[k];
            for(int j=0;j<3;j++) for(int z=0;z<3;z++) next[i][j]+=r[i][z]*b[z][j];
        }
        std::memcpy(r,next,sizeof(r));
    }
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) p[i]+=r[i][j]*offset[j];
    std::fill(out,out+16,0.0); out[15]=1.0;
    for(int i=0;i<3;i++) { for(int j=0;j<3;j++) out[4*i+j]=r[i][j]; out[4*i+3]=p[i]; }
    if(!jac) return;
    for(int k=0;k<7;k++) {
        double delta[3]; for(int i=0;i<3;i++) delta[i]=p[i]-origins[k][i];
        jac[0][k]=axes[k][1]*delta[2]-axes[k][2]*delta[1];
        jac[1][k]=axes[k][2]*delta[0]-axes[k][0]*delta[2];
        jac[2][k]=axes[k][0]*delta[1]-axes[k][1]*delta[0];
        for(int i=0;i<3;i++) jac[i+3][k]=axes[k][i];
    }
}

void so3log(const double r[3][3], double *w) {
    w[0]=(r[2][1]-r[1][2])*.5; w[1]=(r[0][2]-r[2][0])*.5; w[2]=(r[1][0]-r[0][1])*.5;
    double sine=norm3(w), cosine=clamp((r[0][0]+r[1][1]+r[2][2]-1)*.5,-1,1);
    double angle=std::atan2(sine,cosine);
    if(angle<1e-8) return;
    if(PI-angle<1e-5) {
        int index=0; for(int i=1;i<3;i++) if(r[i][i]>r[index][index]) index=i;
        double axis[3]={}; axis[index]=std::sqrt(std::max(0.0,(r[index][index]+1)*.5));
        if(axis[index]<1e-10) { axis[0]=1; axis[1]=axis[2]=0; }
        else for(int i=0;i<3;i++) if(i!=index) axis[i]=(r[i][index]+r[index][i])/(4*axis[index]);
        double length=norm3(axis); for(double &a:axis) a/=length;
        double sign=1;
        if(sine>1e-10) { if(axis[0]*w[0]+axis[1]*w[1]+axis[2]*w[2]<0) sign=-1; }
        else for(double a:axis) if(std::abs(a)>1e-8) { if(a<0) sign=-1; break; }
        for(int i=0;i<3;i++) w[i]=angle*sign*axis[i];
        return;
    }
    for(int i=0;i<3;i++) w[i]*=angle/sine;
}

double error(const double *goal, const double *actual, double *e, double &mm, double &degrees) {
    double rotation[3][3]={}, w[3];
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) for(int k=0;k<3;k++) rotation[i][j]+=goal[4*i+k]*actual[4*j+k];
    so3log(rotation,w);
    for(int i=0;i<3;i++) { e[i]=goal[4*i+3]-actual[4*i+3]; e[i+3]=RS*w[i]; }
    mm=norm3(e); degrees=norm3(w)*DEG;
    double merit=0; for(int i=0;i<6;i++) merit+=e[i]*e[i]; return merit;
}

bool cholsolve(const H &a, const double *rhs, double *x, int n) {
    H l={}; V y={};
    for(int i=0;i<n;i++) for(int j=0;j<=i;j++) {
        double v=a[i][j]; for(int k=0;k<j;k++) v-=l[i][k]*l[j][k];
        if(i==j) { if(!(v>0) || !std::isfinite(v)) return false; l[i][j]=std::sqrt(v); }
        else l[i][j]=v/l[j][j];
    }
    for(int i=0;i<n;i++) { double v=rhs[i]; for(int j=0;j<i;j++) v-=l[i][j]*y[j]; y[i]=v/l[i][i]; }
    for(int i=n-1;i>=0;i--) { double v=y[i]; for(int j=i+1;j<n;j++) v-=l[j][i]*x[j]; x[i]=v/l[i][i]; }
    return true;
}

bool boxqp(const H &h, const double *g, const double *lo, const double *hi, double *x, int n=7) {
    V rhs={}; for(int i=0;i<n;i++) { if(lo[i]>hi[i]) return false; rhs[i]=-g[i]; }
    if(!cholsolve(h,rhs,x,n)) return false;
    int active[7]={}; bool inside=true;
    for(int i=0;i<n;i++) {
        if(x[i]<lo[i] || x[i]>hi[i]) inside=false;
        x[i]=clamp(x[i],lo[i],hi[i]);
        if(x[i]<=lo[i]) active[i]=-1; if(x[i]>=hi[i]) active[i]=1;
        if(hi[i]-lo[i]<=1e-14) active[i]=2;
    }
    if(inside) return true;
    for(int iteration=0;iteration<200;iteration++) {
        int free[7], nf=0; for(int i=0;i<n;i++) if(active[i]==0) free[nf++]=i;
        if(nf) {
            H reduced={}; V b={}, proposed={}, direction={}, ratios={};
            for(int i=0;i<nf;i++) {
                b[i]=-g[free[i]];
                for(int j=0;j<n;j++) if(active[j]) b[i]-=h[free[i]][j]*x[j];
                for(int j=0;j<nf;j++) reduced[i][j]=h[free[i]][free[j]];
            }
            if(!cholsolve(reduced,b,proposed,nf)) return false;
            double amount=1;
            for(int i=0;i<nf;i++) {
                direction[i]=proposed[i]-x[free[i]]; ratios[i]=1;
                if(direction[i]>1e-15) ratios[i]=(hi[free[i]]-x[free[i]])/direction[i];
                else if(direction[i]<-1e-15) ratios[i]=(lo[free[i]]-x[free[i]])/direction[i];
                amount=std::min(amount,ratios[i]);
            }
            amount=clamp(amount,0,1);
            for(int i=0;i<nf;i++) x[free[i]]=clamp(x[free[i]]+amount*direction[i],lo[free[i]],hi[free[i]]);
            if(amount<1-1e-12) {
                for(int i=0;i<nf;i++) if(ratios[i]<=amount+1e-12) {
                    int index=free[i];
                    if(direction[i]>0) { x[index]=hi[index]; active[index]=1; }
                    else { x[index]=lo[index]; active[index]=-1; }
                }
                continue;
            }
        }
        V derivative={}; double maxg=0,maxhx=0,violation=-std::numeric_limits<double>::infinity(); int release=-1;
        for(int i=0;i<n;i++) {
            double hx=0; for(int j=0;j<n;j++) hx+=h[i][j]*x[j]; derivative[i]=hx+g[i];
            maxg=std::max(maxg,std::abs(g[i])); maxhx=std::max(maxhx,std::abs(hx));
            double v=active[i]==-1 ? -derivative[i] : active[i]==1 ? derivative[i] : -std::numeric_limits<double>::infinity();
            if(v>violation) { violation=v; release=i; }
        }
        if(violation<=1e-9*(1+maxg+maxhx)) return true;
        if(release<0) return false; active[release]=0;
    }
    return false;
}

double cap(double x) {
    if(!std::isfinite(x) || x<0 || x>70) return -std::numeric_limits<double>::infinity();
    int lo=0,hi=NB; while(lo<hi) { int mid=(lo+hi)/2; if(XS[mid]<x) lo=mid+1; else hi=mid; }
    return YS[std::min(lo,NB-1)];
}
double clearance(const double *q, const double *reserve, double margin=2) {
    double x=std::abs(q[5])*DEG+margin+reserve[0];
    return std::min(70-x,cap(x)-margin-reserve[1]-std::abs(q[6])*DEG);
}
double pairclear(const double *q, const double *reserve, double margin=2) {
    double x=std::abs(q[0])*DEG+margin+reserve[0];
    return std::min(70-x,cap(x)-margin-reserve[1]-std::abs(q[1])*DEG);
}
bool segment(const double *a, const double *b, const double *reserve, int depth=0) {
    double corner[2]={std::max(std::abs(a[0]),std::abs(b[0])),std::max(std::abs(a[1]),std::abs(b[1]))};
    if(pairclear(corner,reserve)>=-1e-9) return true;
    if(pairclear(a,reserve)<-1e-9 || pairclear(b,reserve)<-1e-9 || depth>=14) return false;
    double mid[2]={(a[0]+b[0])*.5,(a[1]+b[1])*.5};
    return segment(a,mid,reserve,depth+1) && segment(mid,b,reserve,depth+1);
}
bool guard(const double *a, const double *b, const double *velocity, const double *reserve, bool retreat) {
    if(segment(a+5,b+5,reserve)) return true;
    if(!retreat || clearance(a,reserve)>=0) return false;
    const double zero[2]={};
    if(clearance(a,zero,0)<0 || clearance(b,zero,0)<0) return false;
    for(int i=5;i<7;i++) if(a[i]*velocity[i]>1e-10 || std::abs(b[i])>std::abs(a[i])+1e-10 || a[i]*b[i]<-1e-12) return false;
    return true;
}

struct Context {
    const double *reference,*measured,*velocity,*lower,*upper,*offset; double reserve[2]; bool retreat;
    bool safe(const double *q) const {
        if(!finite(q,7)) return false;
        for(int i=0;i<7;i++) if(q[i]<lower[i]-1e-12 || q[i]>upper[i]+1e-12) return false;
        return guard(reference,q,velocity,reserve,retreat) && guard(measured,q,velocity,reserve,retreat);
    }
};
struct Box { V lower,upper; };
int rectangles(const Context &ctx, Box *boxes, bool include_raw) {
    int count=0;
    for(int mode=0;mode<(include_raw?2:1);mode++) for(int row=0;row<NB;row++) {
        double caps[2];
        caps[0]=(XS[row]-(mode?0:2+ctx.reserve[0]))/DEG;
        caps[1]=(YS[row]-(mode?0:2+ctx.reserve[1]))/DEG;
        if(caps[0]<0 || caps[1]<0) continue;
        Box box; copy7(ctx.lower,box.lower); copy7(ctx.upper,box.upper); bool valid=true;
        for(int j=0;j<2;j++) {
            // Only q6 indexes the discontinuous lookup. A q7 equality bound
            // on the existing allowed cap must remain representable. The
            // tiny q7 allowance is below the guard's 1e-9 degree tolerance.
            caps[j]=j==0?std::max(0.0,caps[j]-1e-10):caps[j]+1e-12;
            box.lower[j+5]=std::max(box.lower[j+5],-caps[j]);
            box.upper[j+5]=std::min(box.upper[j+5],caps[j]);
        }
        for(int j=0;j<7;j++) if(box.lower[j]>box.upper[j]) valid=false;
        if(!valid) continue;
        bool duplicate=false;
        for(int k=0;k<count;k++) {
            double delta=0;
            for(int j=5;j<7;j++) delta=std::max(delta,std::max(std::abs(box.lower[j]-boxes[k].lower[j]),std::abs(box.upper[j]-boxes[k].upper[j])));
            if(delta<1e-13) { duplicate=true; break; }
        }
        if(!duplicate) boxes[count++]=box;
    }
    return count;
}
} // namespace

extern "C" {
struct Art7Result {
    int feasible,success,status,iterations,qp_calls;
    double q[7],tcp[16],position_error_mm,orientation_error_deg;
};
const char *art7_wrist_config() { return "5d8bc3b8d8f4e283"; }
int art7_abi_version() { return 1; }
int art7_fk(const double *q,const double *offset,double *tcp,double *jacobian) {
    if(!finite(q,7) || !finite(offset,3)) return -1;
    double j[6][7]; fk(q,offset,tcp,j); if(jacobian) std::memcpy(jacobian,j,sizeof(j)); return 0;
}
int art7_rotation_log(const double *r,double *out) {
    if(!finite(r,9)) return -1; double matrix[3][3]; std::memcpy(matrix,r,sizeof(matrix)); so3log(matrix,out); return 0;
}
int art7_box_qp(const double *h,const double *g,const double *lo,const double *hi,int n,double *out) {
    if(n<1 || n>7 || !finite(h,n*n) || !finite(g,n) || !finite(lo,n) || !finite(hi,n)) return -1;
    H matrix={}; for(int i=0;i<n;i++) for(int j=0;j<n;j++) {
        if(std::abs(h[i*n+j]-h[j*n+i])>1e-10+1e-12*std::abs(h[j*n+i])) return -1;
        matrix[i][j]=h[i*n+j];
    }
    return boxqp(matrix,g,lo,hi,out,n)?0:1;
}

int art7_run(const double *goal,const double *reference,const double *measured,
             const double *velocity,const double *reserve_velocity,const double *lower,
             const double *upper,const double *offset,int max_iterations,double trust_radius,
             int allow_retreat,Art7Result *result) {
    if(!result || max_iterations<1 || !(trust_radius>0) || !finite(goal,16) || !finite(reference,7)
       || !finite(measured,7) || !finite(velocity,7) || !finite(reserve_velocity,7)
       || !finite(lower,7) || !finite(upper,7) || !finite(offset,3)) return -1;
    *result=Art7Result{};
    Context ctx{reference,measured,velocity,lower,upper,offset,
                {std::max(std::abs(velocity[5]),std::abs(reserve_velocity[5]))*DEG*.2,
                 std::max(std::abs(velocity[6]),std::abs(reserve_velocity[6]))*DEG*.2},allow_retreat!=0};
    bool outside=clearance(reference,ctx.reserve)<-1e-9 || clearance(measured,ctx.reserve)<-1e-9;
    if((outside&&!ctx.retreat) || !guard(reference,reference,velocity,ctx.reserve,ctx.retreat)
       || !guard(measured,measured,velocity,ctx.reserve,ctx.retreat)) { result->status=4; return 0; }
    for(int i=0;i<7;i++) if(lower[i]>upper[i]) { result->status=5; return 0; }
    Box boxes[2*NB]; int box_count=rectangles(ctx,boxes,outside&&ctx.retreat);
    // Even if numerical construction yields no rectangles, the ordinary box
    // candidate may satisfy the exact guard (e.g. an exact boundary hold).
    V q,best; copy7(reference,q); copy7(reference,best);
    double tcp[16],jac[6][7],residual[6],mm=0,deg=0;
    fk(reference,offset,tcp,nullptr);
    double reference_merit=error(goal,tcp,residual,mm,deg);
    bool has_best=ctx.safe(q);
    double best_merit=has_best?reference_merit:std::numeric_limits<double>::infinity(),damping=.01;
    int iteration=0,calls=0;
    for(;iteration<max_iterations;iteration++) {
        fk(q,offset,tcp,jac); double current_merit=error(goal,tcp,residual,mm,deg);
        if(mm<=.02 && deg<=.01 && ctx.safe(q)) { copy7(q,best); has_best=true; break; }
        for(int i=3;i<6;i++) for(int j=0;j<7;j++) jac[i][j]*=RS;
        H h={}; V g={},local_lo,local_hi; bool bounds_valid=true;
        for(int i=0;i<7;i++) {
            for(int j=0;j<7;j++) { for(int k=0;k<6;k++) h[i][j]+=jac[k][i]*jac[k][j]; if(i==j) h[i][j]+=damping; }
            for(int k=0;k<6;k++) g[i]-=jac[k][i]*residual[k];
            local_lo[i]=std::max(lower[i],q[i]-trust_radius); local_hi[i]=std::min(upper[i],q[i]+trust_radius);
            if(local_lo[i]>local_hi[i]) bounds_valid=false;
        }
        if(!bounds_valid) break;
        V candidates[2*NB+1]; int candidate_count=0;
        V delta_lo,delta_hi,delta,ordinary;
        for(int i=0;i<7;i++) { delta_lo[i]=local_lo[i]-q[i]; delta_hi[i]=local_hi[i]-q[i]; }
        calls++;
        if(boxqp(h,g,delta_lo,delta_hi,delta)) {
            for(int i=0;i<7;i++) ordinary[i]=q[i]+delta[i];
            if(ctx.safe(ordinary)) copy7(ordinary,candidates[candidate_count++]);
        }
        if(!candidate_count) {
            double keys[2*NB][4]; int key_count=0;
            for(int b=0;b<box_count;b++) {
                V lo,hi; bool valid=true;
                for(int i=0;i<7;i++) {
                    lo[i]=std::max(local_lo[i],boxes[b].lower[i]); hi[i]=std::min(local_hi[i],boxes[b].upper[i]);
                    if(lo[i]>hi[i]) valid=false;
                }
                if(!valid) continue;
                double key[4]={lo[5],lo[6],hi[5],hi[6]}; bool duplicate=false;
                for(int k=0;k<key_count;k++) {
                    double diff=0; for(int j=0;j<4;j++) diff=std::max(diff,std::abs(key[j]-keys[k][j]));
                    if(diff<1e-13) { duplicate=true; break; }
                }
                if(duplicate) continue; std::copy(key,key+4,keys[key_count++]);
                for(int i=0;i<7;i++) { delta_lo[i]=lo[i]-q[i]; delta_hi[i]=hi[i]-q[i]; }
                calls++;
                if(!boxqp(h,g,delta_lo,delta_hi,delta)) continue;
                for(int i=0;i<7;i++) ordinary[i]=q[i]+delta[i];
                if(ctx.safe(ordinary)) copy7(ordinary,candidates[candidate_count++]);
            }
        }
        V next; bool have_next=false; double next_merit=std::numeric_limits<double>::infinity();
        for(int c=0;c<candidate_count;c++) for(double alpha=1;alpha>=.0625;alpha*=.5) {
            V trial; for(int i=0;i<7;i++) trial[i]=q[i]+alpha*(candidates[c][i]-q[i]);
            if(!ctx.safe(trial)) continue;
            fk(trial,offset,tcp,nullptr); double merit=error(goal,tcp,residual,mm,deg);
            if(merit<next_merit) { copy7(trial,next); next_merit=merit; have_next=true; }
            if(merit<=current_merit) break;
        }
        if(!have_next) break;
        if(next_merit<best_merit) { copy7(next,best); best_merit=next_merit; has_best=true; }
        if(next_merit>=current_merit-1e-14 && ctx.safe(q)) {
            damping*=10; double change=0; for(int i=0;i<7;i++) change=std::max(change,std::abs(next[i]-q[i]));
            if(damping>1e5 || change<1e-11) break;
            continue;
        }
        copy7(next,q); damping=std::max(.001,damping*.5);
    }
    result->iterations=std::min(iteration+1,max_iterations); result->qp_calls=calls;
    if(!has_best || !ctx.safe(best)) { result->status=7; return 0; }
    result->feasible=1; copy7(best,result->q); fk(best,offset,result->tcp,nullptr);
    error(goal,result->tcp,residual,result->position_error_mm,result->orientation_error_deg);
    result->success=result->position_error_mm<=.02 && result->orientation_error_deg<=.01;
    if(result->success) { result->status=0; if(iteration<max_iterations) result->iterations=iteration; }
    else {
        double change=0; for(int i=0;i<7;i++) change=std::max(change,std::abs(best[i]-reference[i]));
        result->status=change>1e-11?1:2;
        if(best_merit>reference_merit+1e-12) result->status=3;
    }
    return 0;
}
} // extern C
