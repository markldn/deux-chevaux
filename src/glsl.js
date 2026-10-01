// All GLSL. Lit shaders output linear HDR radiance; one post pass does bloom, DOF, grade and tonemap.
export const HEAD = `#version 300 es
precision highp float;precision highp int;precision highp sampler2DShadow;
`;
export const COMMON = `
uniform mat4 uVP,uSh0M,uSh1M;
uniform vec3 uCam,uSun,uSunC,uSkyZ,uSkyH,uGndC;
uniform float uT,uFogD,uShOn;
uniform sampler2DShadow uSh0,uSh1;
float hash(ivec2 p){uint h=(uint(p.x)*1597334677u)^(uint(p.y)*3812015801u);h^=h>>16;h*=2246822519u;h^=h>>13;h*=3266489917u;h^=h>>16;return float(h)*(1./4294967296.);}
float vnoise(vec2 p){vec2 i=floor(p),f=p-i,u=f*f*f*(f*(f*6.-15.)+10.);ivec2 q=ivec2(i);
return mix(mix(hash(q),hash(q+ivec2(1,0)),u.x),mix(hash(q+ivec2(0,1)),hash(q+ivec2(1,1)),u.x),u.y);}
float fbm(vec2 p,int o){float s=0.,a=.5;for(int i=0;i<o;i++){s+=a*vnoise(p);p=vec2(1.6*p.x+1.2*p.y+17.3,-1.2*p.x+1.6*p.y+9.1);a*=.5;}return s;}
float n3(vec3 p){return vnoise(p.xy*1.3+p.z*vec2(17.31,-9.7));}
float boxd(vec2 p,vec4 r){return length(vec2(max(max(r.x-p.x,0.),p.x-r.y),max(max(r.z-p.y,0.),p.y-r.w)));}
const vec4 FAC=vec4(-70,70,-300,60),FIELD=vec4(90,230,-170,-30);const float AVE=140.;
float roadM(vec2 p){return max(1.-smoothstep(3.5,7.,abs(p.y-AVE)),(p.y>50.&&p.y<AVE)?1.-smoothstep(3.,6.,abs(p.x)):0.);}
float plough(vec2 p){return boxd(p,FIELD)>0.?0.:.075*sin(p.x*8.378)*smoothstep(0.,4.,min(min(p.x-FIELD.x,FIELD.y-p.x),min(p.y-FIELD.z,FIELD.w-p.y)));}
float hgt(vec2 p){float d=length(p+vec2(0,100));
 float h=(fbm(p/520.,4)-.5)*34.*smoothstep(80.,600.,d);
 h+=((fbm(p/90.+vec2(5,0),3)-.5)*4.+(vnoise(p/7.)-.5)*.25)*(1.-roadM(p));
 h*=smoothstep(0.,45.,boxd(p,FAC));return h;}
float hgtP(vec2 p){return hgt(p)+plough(p);}
mat2 rot(float a){float c=cos(a),s=sin(a);return mat2(c,s,-s,c);}
`;
export const LIGHT = `
out vec4 oC;
#ifndef SH
float shTap(vec3 Q,mat4 M,float b,int k){vec4 c=M*vec4(Q,1);c.xyz=c.xyz*.5+.5;
 if(any(greaterThan(abs(c.xy-.5),vec2(.495)))||c.z>1.)return -1.;
 float s=0.;for(int i=-1;i<=1;i++)for(int j=-1;j<=1;j++)s+=k==0?texture(uSh0,vec3(c.xy+vec2(i,j)*b,c.z-.0003)):texture(uSh1,vec3(c.xy+vec2(i,j)*b,c.z-.0008));
 return s/9.;}
#endif
float shadowF(vec3 P,vec3 N){
#ifdef SH
 return 1.;
#else
 if(uShOn<.5)return 1.;
 vec4 c=uSh0M*vec4(P,1);float e=max(abs(c.x),abs(c.y));
 float s1=shTap(P+N*.08,uSh1M,1./2048.,1);if(s1<0.)s1=1.;
 if(e<.98){float s0=shTap(P+N*.012,uSh0M,.7/4096.,0);if(s0>=0.)return mix(s0,s1,smoothstep(.8,.98,e));}
 return s1;
#endif
}
// sky + a dark tree-line band at the horizon and lit ground below it: what the paint reflects
vec3 sky(vec3 d){float mu=max(dot(d,uSun),0.);
 vec3 c=mix(uSkyH,uSkyZ,pow(clamp(d.y,0.,1.),.5))+uSunC*(.04*pow(mu,5.)+.18*pow(mu,48.));
 return c;}
vec3 env(vec3 d,float r){
 vec3 s=sky(d);float az=atan(d.z,d.x);
 float tl=.012+.035*fbm(vec2(az*9.,1.),3);
 vec3 trees=uSkyH*.25+uGndC*.15;
 vec3 g=uGndC*(.55+.45*max(uSun.y,0.))*(.8+.2*vnoise(vec2(az*20.,d.y*30.)));
 vec3 c=d.y>tl?s:d.y>0.?mix(trees,s,smoothstep(tl*.6,tl,d.y)*r):g;
 vec3 amb=mix(uGndC*.5,mix(uSkyH,uSkyZ,.5),smoothstep(-.2,.3,d.y));
 return mix(c,amb,r*r);}
void fog(inout vec3 h,vec3 P){vec3 v=P-uCam;float d=length(v);v/=d;
 float k=1.-exp(-d*uFogD*(1.+1.5*exp(-max(P.y,0.)*.03)));
 float s=pow(max(dot(v,uSun),0.),8.);
 h=mix(h,uSkyH*1.02+uSunC*s*.25,k);}
float ggx(float nh,float a){float a2=a*a;float d=nh*nh*(a2-1.)+1.;return a2/(3.14159*d*d);}
// albedo, roughness, metalness, ambient occlusion, clearcoat, emission
vec3 shade(vec3 P,vec3 N,vec3 alb,float rough,float metal,float ao,float cc,vec3 em){
#ifdef SH
 return vec3(1);
#else
 vec3 V=normalize(uCam-P),L=uSun,H=normalize(L+V);float sh=shadowF(P,N);
 float nl=max(dot(N,L),0.),nv=max(dot(N,V),1e-3),nh=max(dot(N,H),0.),vh=max(dot(V,H),0.);
 float a=max(rough*rough,.002);vec3 F0=mix(vec3(.04),alb,metal);
 vec3 F=F0+(1.-F0)*pow(1.-vh,5.);float k=a*.5+.0001;float G=nv/(nv*(1.-k)+k)*nl/(nl*(1.-k)+k);
 vec3 spec=F*ggx(nh,a)*G/max(4.*nv*nl,1e-3);
 vec3 dif=alb*(1.-metal)/3.14159;
 vec3 col=(dif*3.14159+spec*3.14159)*uSunC*nl*sh;
 vec3 Fv=F0+(1.-F0)*pow(1.-nv,5.)/(1.+4.*rough);
 vec3 R=reflect(-V,N);float occ=ao*mix(1.,smoothstep(-.35,.15,R.y)*.6+.4,1.-ao);
 vec3 amb=mix(uGndC*.6,mix(uSkyH,uSkyZ,.55),N.y*.5+.5)*ao;
 col+=dif*3.14159*amb*.9+env(R,rough)*Fv*occ;
 if(cc>0.){float fc=.04+.96*pow(1.-nv,5.);col=col*(1.-fc*cc)+cc*fc*env(R,.02)*occ+cc*.25*ggx(nh,.03)*nl*sh*uSunC*fc*4.;}
 col+=em;
 fog(col,P);return min(col,vec3(48.));
#endif
}
`;
// ---------------------------------------------------------------- sky
export const SKY_VS = `in vec3 aP;out vec2 vP;void main(){vP=aP.xy;gl_Position=vec4(aP.xy,1,1);}`;
export const SKY_FS = `in vec2 vP;uniform mat4 uIVP;
void main(){vec4 w=uIVP*vec4(vP,1,1);vec3 d=normalize(w.xyz/w.w-uCam);
 vec3 h=sky(d);float mu=dot(d,uSun);
 h+=uSunC*smoothstep(.9998,.99992,mu)*40.;
 if(d.y>0.){vec2 cp=d.xz/(d.y+.08)*1200.+vec2(uT*4.,uT*1.5);
  float c=fbm(cp/1500.,5),c2=fbm(cp/1500.+uSun.xz*.05,5);
  float a=smoothstep(.5,.78,c)*smoothstep(0.,.15,d.y);
  float lit=clamp(.65+(c-c2)*5.,.2,1.4);
  h=mix(h,uSunC*.3*lit+uSkyZ*.5+uSkyH*.2,a*.9);}
 oC=vec4(h,1);}`;
// ---------------------------------------------------------------- terrain
export const TERR_VS = `in vec3 aP;uniform vec2 uC;out vec3 vP;out vec3 vN;
void main(){vec2 g=sign(aP.xy)*pow(abs(aP.xy),vec2(3.))*2400.;vec2 p=uC+g;float h=hgt(p);
 float e=.04+length(g)*.01;vec3 n=normalize(vec3(hgt(p-vec2(e,0))-hgt(p+vec2(e,0)),2.*e,hgt(p-vec2(0,e))-hgt(p+vec2(0,e))));
 vP=vec3(p.x,h,p.y);vN=n;gl_Position=uVP*vec4(vP,1);}`;
export const TERR_FS = `in vec3 vP;in vec3 vN;uniform vec4 uSkid[16];
// ground: patchwork fields, avenue, test-centre concrete, runway, camera pit lid
void main(){vec2 p=vP.xz;float d=length(vP-uCam);vec3 N=normalize(vN);
 if(p.x>-1.3&&p.x<1.3&&p.y>-4.6&&p.y<-.6)discard; // camera pit (drawn by the pit mesh)
 float n1=vnoise(p*.07),n2=vnoise(p*1.7),n3=vnoise(p*13.),n4=fbm(p*.011,3);
 // field cells
 vec2 cq=rot(.35)*p/vec2(130,95);ivec2 ci=ivec2(floor(cq));float ft=hash(ci+ivec2(7,3));vec2 cf=fract(cq);
 float hedge=smoothstep(.012,.0,min(min(cf.x,1.-cf.x)*130.,min(cf.y,1.-cf.y)*95.)/130.);
 vec3 grass=mix(vec3(.11,.17,.04),vec3(.25,.27,.08),n4)*(.8+.3*n2);
 vec3 c=grass;float rough=.9;
 float rows=sin(dot(p,rot(.35)*vec2(0,1))*7.);
 if(ft<.25)c=mix(vec3(.55,.42,.17),vec3(.66,.52,.24),n2)*(.92+.08*rows);          // wheat stubble
 else if(ft<.38)c=mix(vec3(.2,.13,.08),vec3(.3,.2,.12),n2)*(.85+.15*rows);         // ploughed
 else if(ft<.47)c=mix(vec3(.28,.2,.42),vec3(.2,.26,.12),smoothstep(-.3,.3,rows));  // lavender rows
 else if(ft<.55)c=mix(vec3(.35,.33,.06),vec3(.12,.2,.04),smoothstep(-.2,.6,rows)); // sunflowers
 c=mix(c,vec3(.06,.1,.03),hedge*.8);
 // the ploughed test field: furrows you can see and feel
 float fm=boxd(p,FIELD);
 if(fm<=0.){float f=sin(p.x*8.378),e=smoothstep(0.,4.,-fm);c=mix(vec3(.13,.09,.055),vec3(.34,.24,.15),f*.5+.5)*(.8+.4*n3);rough=1.;N=normalize(N+vec3(-.63*cos(p.x*8.378)*e,0,0)+vec3(n2-.5,0,n3-.5)*.25);}
 // facility concrete + runway
 float fac=boxd(p,FAC);
 if(fac<=0.){vec3 con=vec3(.27,.265,.25)*(.85+.15*n2)*(.9+.1*n3);
  vec2 sl=abs(fract(p/vec2(6.,6.))-.5);float joint=smoothstep(.006,.0,min(sl.x,sl.y)*6.)*.4;
  con*=1.-joint;float runway=step(abs(p.x),4.5)*step(p.y,0.);
  vec3 asp=vec3(.09,.09,.09)*(.8+.4*n3);
  vec3 cc=mix(con,asp,runway);
  if(runway>0.){cc=mix(cc,vec3(.75,.72,.6),step(abs(abs(p.x)-4.1),.08)+step(abs(p.x),.07)*step(fract(p.y/6.),.5));
   float m=abs(fract(p.y/10.)-.5);if(p.y>-60.)cc=mix(cc,vec3(.8,.66,.1),step(m,.04)*step(abs(p.x),3.6)*.8);}
  // tow-cable guide rail in the runway
  cc=mix(cc,vec3(.25,.25,.27),step(abs(p.x-.0),.035)*step(p.y,-.6)*runway);
  c=mix(c,cc,1.-smoothstep(0.,1.,fac));rough=.85;}
 // roads
 float rm=roadM(p);if(rm>0.){vec3 r=vec3(.11,.11,.11)*(.8+.3*n3)*(.9+.2*n2);
  float cl=abs(p.y-AVE)<4.&&abs(p.x)>5.?step(abs(p.y-AVE),.06)*step(fract(p.x/9.),.5):0.;
  r=mix(r,vec3(.8),cl*.8);c=mix(c,r,rm);rough=mix(rough,.75,rm);}
 // tyre marks
 for(int i=0;i<16;i++){vec4 s=uSkid[i];if(s.w<=0.)continue;}
 // close-up grain
 if(d<30.){c*=.85+.3*vnoise(p*40.)*(1.-d/30.);}
 vec3 col=shade(vP,N,c,rough,0.,1.,0.,vec3(0));oC=vec4(col,1);}`;
// ---------------------------------------------------------------- car (and every skinned/boned mesh)
export const CAR_VS = `in vec3 aP;in vec3 aN;in vec4 aC;in vec4 aM;in vec4 aJ;in vec4 aW;in vec4 aK;
uniform mat4 uB[24],uA[3];uniform float uExplode,uDum;uniform vec3 uHub[4];uniform highp sampler2D uNP,uNQ,uNR;
out vec3 vW;out vec3 vN;out vec3 vR;out vec4 vC;out vec4 vM;out vec4 vK;out vec3 vL;
vec3 qr(vec4 q,vec3 v){vec3 t=2.*cross(q.xyz,v);return v+q.w*t+cross(q.xyz,t);}
void main(){int b=int(aM.y+.5);vec3 p=aP,n=aN;float dm=0.;
 if(uDum<.5&&b>=5&&b<=7){mat4 A=uA[b-5];p=(A*vec4(p,1)).xyz;n=mat3(A)*n;}
 // node skinning: each vertex follows 4 lattice nodes, carried by their positions and local rotations
 if(aW.x>0.){vec3 sp=vec3(0),sn=vec3(0);
  for(int k=0;k<4;k++){float w=aW[k];if(w<=0.)continue;int id=int(aJ[k]+.5);ivec2 c=ivec2(id%512,id/512);
   vec4 P=texelFetch(uNP,c,0);vec4 Q=texelFetch(uNQ,c,0);vec3 R=texelFetch(uNR,c,0).xyz;
   sp+=w*(P.xyz+qr(Q,p-R));sn+=w*qr(Q,n);dm+=w*P.w;}
  p=sp;n=sn;}
 float part=aM.w;
 if(uExplode>0.){float e=uExplode;
  if(part<.5)p.y+=e*1.25;else if(part<1.5)p.y-=e*.05;else if(part<2.5){p.y+=e*.45;p.z+=e*.55;}else if(part<3.5)p.y-=e*.18;else if(part<4.5)p.x+=sign(p.x)*e*.45;else p.y+=e*.62;}
 vec4 w=uB[b]*vec4(p,1);vW=w.xyz;vN=mat3(uB[b])*n;vR=aP;vC=aC;vM=vec4(aM.x,aM.z,part,dm);vK=aK;
 vL=b>0&&b<5?aP-uHub[b-1]:vec3(0);
 gl_Position=uVP*w;}`;
export const CAR_FS = `in vec3 vW;in vec3 vN;in vec3 vR;in vec4 vC;in vec4 vM;in vec4 vK;in vec3 vL;
uniform vec3 uPaint,uPaint2;uniform float uTwo,uMarks,uLights,uBrake,uXray,uRoof,uCrack,uDirt;uniform vec3 uCrackP;
float ring(vec2 p,vec2 c,float r){vec2 d=p-c;float l=length(d);if(l>r)return -1.;return (d.x*d.y>0.)==(l<r*.55)?1.:0.;}
void main(){
 int mat=int(vM.x+.5);float ao=vM.y,dmg=vM.w;vec3 N=normalize(vN);bool ff=gl_FrontFacing;if(!ff)N=-N;
 vec3 R=vR;vec3 alb=vC.rgb;float rough=vC.a,metal=0.,cc=0.;vec3 em=vec3(0);
 float win=vK.x,seam=vK.y,canv=vK.z,s=vK.w;
#ifdef GLASS
 if(uXray>0.&&vM.z<.5){vec3 V=normalize(uCam-vW);float f=pow(1.-abs(dot(N,V)),3.);oC=vec4(vec3(.3,.75,1.)*(.08+1.6*f)*uXray,(.05+.5*f)*uXray);return;}
 if(mat!=0||win>0.)discard;
#else
 if(mat==0&&win<0.)discard;
 if(uXray>0.&&vM.z<.5)discard;
#endif
 if(mat==0){ // body paint and everything painted onto the shell
  float roofEdge=mix(.37,-1.52,uRoof);
  if(canv<0.&&R.z>roofEdge&&uRoof>0.){discard;}
  alb=uPaint;rough=.32;cc=1.;
  if(uTwo>0.){float sw=R.y-(.72+.32*smoothstep(-.1,.75,-R.z*.6+.2)-.25*smoothstep(.4,1.2,R.z));if((sw<0.&&abs(s)>.5)||(R.z>.705&&abs(s)<.8)||R.y<.36)alb=uPaint2;}
  // grime low down, faded paint on top
  float gr=smoothstep(.62,.3,R.y)*(.5+.5*n3(R*9.))*uDirt;alb=mix(alb,vec3(.16,.13,.1),gr*.5);rough=mix(rough,.8,gr);
  if(win<.014&&win>-.0){alb=vec3(.015);rough=.6;cc=0.;}                     // window rubber
  if(seam<.0022){alb*=.08;rough=.9;cc=0.;}                                   // panel gaps
  else if(seam<.006){N=normalize(N+vec3(0,.0,0));alb*=.82;}
  if(canv<0.){float st=abs(fract(s*4.+.5)-.5);alb=vec3(.035,.035,.04)*(.9+.2*vnoise(R.zx*vec2(300.,60.)));rough=.92;cc=0.;
   alb*=1.-smoothstep(.03,.0,st)*.5;N=normalize(N+vec3(0,0,1)*.05*sin(R.z*40.));}
  if(R.z>1.765&&N.z>.3&&!(!ff)){ // nose: grille, chevrons
   float gx=abs(R.x),gy=R.y;
   if(gx<.27&&gy>.5&&gy<.735){float sl=fract((gy-.5)/.026);alb=mix(vec3(.01),vec3(.12),smoothstep(.55,.75,sl)*smoothstep(1.,.85,sl));rough=.6;cc=0.;metal=0.;}
   else if(gx<.29&&gy>.48&&gy<.755){alb=vec3(.35);rough=.4;}
   vec2 q=vec2(gx,gy-.775);for(int i=0;i<2;i++){float y0=float(i)*.022;float c1=abs(q.y-y0-(.045-q.x*.42))-.0055;if(c1<0.&&q.x<.075){alb=vec3(.85,.82,.7);metal=1.;rough=.2;cc=0.;}}}
  if(R.z<-1.9&&abs(R.x)<.25&&R.y>.42&&R.y<.52&&ff){alb=vec3(.85,.7,.12);rough=.5;cc=0.;if(abs(R.y-.47)<.03&&abs(fract(R.x*18.)-.5)<.3&&abs(R.x)<.21)alb=vec3(.03);}
  if(!ff){alb=uPaint*.55;rough=.7;cc=0.;
   if(abs(R.x)>.55&&R.y>.38&&R.y<1.02&&R.z<.56&&R.z>-.98)alb=vec3(.16,.15,.14)*(.9+.1*vnoise(R.zy*80.));}
  if(uMarks>0.&&ff){ // crash-test targets: yellow/black quadrant discs
   float m=-1.;if(abs(s)>.6){m=max(m,ring(R.zy,vec2(1.2,.92),.05));m=max(m,ring(R.zy,vec2(-1.2,.88),.05));m=max(m,ring(R.zy,vec2(0.,1.2),.04));m=max(m,ring(R.zy,vec2(.45,.6),.04));}
   else{m=max(m,ring(vec2(s,R.z),vec2(0.,-.4),.07));m=max(m,ring(vec2(s,R.z),vec2(0.,1.2),.06));}
   if(m>=0.){alb=m>.5?vec3(.9,.75,.03):vec3(.01);rough=.5;cc=.5;}}
  // crumpled metal: folds across the crush direction, darker in the troughs, paint flaking on the crests
  if(dmg>.015){float k=clamp(dmg*2.2,0.,1.);vec3 q=R*4.;float wv=n3(q)*3.;
   float f=R.z*24.+R.y*7.+wv,s1=sin(f),c1=cos(f);vec3 q2=R*11.;
   N=normalize(N+k*(.42*vec3(c1*.15,c1*.25,c1)+.22*(vec3(n3(q2),n3(q2+5.),n3(q2+9.))-.5)));
   alb*=1.-k*.3*smoothstep(.2,-.6,s1);
   float ch=smoothstep(.93,.99,s1)*smoothstep(.5,.62,n3(R*40.))*k;alb=mix(alb,vec3(.3,.29,.27),ch);
   cc*=1.-k*.6;rough=mix(rough,.6,k*.7);}
 }
 else if(mat==1){metal=1.;cc=0.;alb=vC.rgb;}
 else if(mat==2){ // tyre: tread pattern on the crown, moulded rings on the sidewall
  float a=atan(vL.y,vL.z),r=length(vL.yz),ax=abs(vL.x);
  if(r>.282){float zz=fract(a*30./3.14159+(ax>.02?.25:0.)*sign(vL.x));float g=step(abs(ax-.022),.004)+step(.88,zz)*step(ax,.048);alb*=1.-g*.6;N=normalize(N+vec3(0,1,0)*0.);rough=.9;}
  else if(r>.22&&r<.25)alb*=1.+.25*step(.6,fract(r*300.));}
 else if(mat==4){alb=vec3(.035);rough=.92;}
 else if(mat==6){float st=step(.5,fract(R.x*14.));alb=mix(vec3(.28,.25,.21),vec3(.17,.16,.15),st)*(.85+.3*vnoise(R.xz*400.));rough=.95;}
 else if(mat==7){metal=.75;rough=.42+.2*n3(R*80.);alb=vec3(.58,.58,.56)*(.85+.2*n3(R*30.));}
 else if(mat==8){rough=.75;alb*=.8+.4*n3(R*20.);alb=mix(alb,vec3(.22,.1,.05),smoothstep(.65,.85,n3(R*7.))*.6);}
 else if(mat==9){metal=0.;rough=.05;cc=1.;
  float head=step(1.5,R.z);float tail=step(R.z,-1.7)*step(.62,R.y);
  float l=head*uLights*(1.+3.*smoothstep(.05,.0,length(R.xy-vec2(sign(R.x)*.52,.93))))+tail*(uLights*.4+uBrake*2.5)+step(abs(R.z-.43),.05)*.4;
  em=vC.rgb*l*vec3(4.,3.6,3.)+vec3(0);alb=vC.rgb*.3;
  float fr=fract(length(R.xy-vec2(sign(R.x)*.52,.93))*90.);alb*=.85+.3*fr*head;}
 else if(mat==10){metal=.2;rough=.35;cc=.4;}
 else if(mat==11){rough=.45;cc=.3;if(vK.w>.5&&abs(R.x-.33)>.045){float m=ring(R.zy,vec2(-.15,1.31),.045);if(m>=0.)alb=m>.5?vec3(.9,.75,.03):vec3(.01);}}
 else if(mat==12){metal=.6;rough=.6;alb=mix(vec3(.25,.24,.23),vec3(.3,.14,.06),n3(R*30.));}
 else if(mat==13){alb=vec3(.07,.07,.075)*(.9+.2*vnoise(R.xz*300.));rough=.75;}
#ifdef GLASS
 { // glass: reflection over a faint tint; cracks spider out from the impact point
  vec3 V=normalize(uCam-vW);float nv=abs(dot(N,V));float f=.04+.96*pow(1.-nv,5.);
  vec3 rr=reflect(-V,N);vec3 c=env(rr,.0)*f+uSunC*pow(max(dot(rr,uSun),0.),900.)*8.;
  float a=.1+f*.85;vec3 tint=vec3(.06,.08,.07);
  if(R.z<.4&&R.z>-.34&&abs(R.y-1.25)<.012)c=vec3(.03),a=1.;   // flap window split
  if(uCrack>0.){float dd=length(R-uCrackP);vec2 q=R.zy*30.+R.x*20.;float an=atan(R.y-uCrackP.y,R.x-uCrackP.x+R.z-uCrackP.z);
   float web=smoothstep(.06,.0,abs(fract(an*5.+vnoise(vec2(dd*20.,an*4.))*.8)-.5))*smoothstep(uCrack*.9,.0,dd)+smoothstep(.04,.0,abs(fract(dd*14.)-.5))*step(dd,uCrack*.7)*.6;
   c+=vec3(.6)*web*uCrack;a=max(a,web*uCrack*.8);}
  fog(c,vW);oC=vec4(c+tint*a*.2,a);return;}
#endif
 vec3 col=shade(vW,N,alb,rough,metal,ao,cc,em);oC=vec4(col,1);}`;
export const SH_FS = `void main(){}`;
// ---------------------------------------------------------------- generic static mesh (barrier, buildings, props)
export const STAT_VS = `in vec3 aP;in vec3 aN;in vec4 aC;in vec4 aM;uniform mat4 uM;out vec3 vW;out vec3 vN;out vec4 vC;out vec4 vM;out vec3 vR;
void main(){vec4 w=uM*vec4(aP,1);vW=w.xyz;vR=aP;vN=mat3(uM)*aN;vC=aC;vM=aM;gl_Position=uVP*w;}`;
export const STAT_FS = `in vec3 vW;in vec3 vN;in vec4 vC;in vec4 vM;in vec3 vR;
void main(){vec3 N=normalize(vN);if(!gl_FrontFacing)N=-N;int m=int(vM.x+.5);vec3 a=vC.rgb;float r=vC.a,mt=0.;vec3 em=vec3(0);
 if(m==20){a*=.85+.3*n3(vR*8.);a*=.9+.1*n3(vR*60.);}  // concrete
 if(m==21){float st=step(.5,fract((vR.x+vR.y)*2.5));a=mix(vec3(.85,.65,.03),vec3(.02),st);r=.5;} // hazard stripes
 if(m==22){mt=1.;}                                   // steel
 if(m==23){vec2 h=vR.xy*vec2(40.,46.);vec2 f=abs(fract(h+vec2(.5*floor(h.y),0.))-.5);a=mix(vec3(.75,.76,.78),vec3(.3),smoothstep(.42,.5,max(f.x,f.y)));mt=.8;r=.4;} // honeycomb
 if(m==24){em=a*6.;}                                 // lamp
 if(m==25){float c=step(.5,fract(vR.x*.8));a*=.8+.2*c;}  // corrugated wall
 if(m==27){float b=smoothstep(.45,.55,vnoise(vec2(atan(vR.x,vR.z)*3.,vR.y*2.5)+vnoise(vR.xy*9.)));a=mix(vec3(.4,.38,.3),vec3(.66,.63,.5),b)*(.8+.3*vnoise(vR.xy*40.));r=.9;}
 if(m==28){float st=fract(vR.x*14.+vnoise(vR.yz*20.));a*=.75+.4*st*vnoise(vR.yz*60.);r=1.;}
 if(m==26){vec3 V=normalize(uCam-vW);float f=.04+.96*pow(1.-abs(dot(N,V)),5.);vec3 c=env(reflect(-V,N),.05)*f;fog(c,vW);oC=vec4(c,.15+f);return;}
 vec3 c=shade(vW,N,a,r,mt,vM.z,0.,em);oC=vec4(c,1);}`;
// ---------------------------------------------------------------- trees (instanced): trunk + canopy blobs, mottled plane-tree bark
export const TREE_VS = `in vec3 aP;in vec3 aN;in vec4 aC;in vec4 aM;in vec4 iA;out vec3 vW;out vec3 vN;out vec4 vC;out vec4 vM;out vec3 vR;
void main(){float s=iA.z;int k=int(iA.w);vec3 p=aP*s;if(k==1&&aM.x>30.5)p.xz*=.45,p.y*=1.25;
 float a=hash(ivec2(iA.xy*10.))*6.28;p.xz=rot(a)*p.xz;
 if(aM.x>30.5){float w=sin(uT*1.3+iA.x*.3)*.04*p.y/10.;p.x+=w;}
 vec3 base=vec3(iA.x,hgt(iA.xy)-.1,iA.y);vW=base+p;vR=aP+vec3(iA.x,0,iA.y);vN=aN;vN.xz=rot(a)*vN.xz;vC=aC;vM=aM;vM.w=float(k);gl_Position=uVP*vec4(vW,1);}`;
export const TREE_FS = `in vec3 vW;in vec3 vN;in vec4 vC;in vec4 vM;in vec3 vR;
void main(){vec3 N=normalize(vN);vec3 a=vC.rgb;float r=.9;
 if(vM.x<30.5){float m=smoothstep(.45,.55,vnoise(vR.xy*vec2(5.,3.)+vR.z*2.));a=mix(vec3(.42,.4,.32),vec3(.62,.6,.48),m)*(.8+.3*vnoise(vR.xy*40.));if(vM.w>.5)a=vec3(.25,.22,.18);}
 else{float l=vnoise(vR.xy*3.+vR.z*7.)*.6+vnoise(vR.xy*11.)*.4;
  #ifndef SH
  if(l<.3)discard;
  #endif
  a=mix(vec3(.05,.09,.02),vec3(.17,.22,.05),l)*(vM.w>.5?.8:1.);N=normalize(N+vec3(l-.5,0,vnoise(vR.zy*9.)-.5));}
 vec3 c=shade(vW,N,a,r,0.,vM.z,0.,vec3(0));oC=vec4(c,1);}`;
// ---------------------------------------------------------------- particles (glass, debris, dust, egg)
export const PART_VS = `in vec4 aP;in vec4 aC;uniform float uSz;out vec4 vC;out float vK;
void main(){gl_Position=uVP*vec4(aP.xyz,1);gl_PointSize=clamp(uSz*aP.w/gl_Position.w,1.,64.);vC=aC;vK=aP.w;}`;
export const PART_FS = `in vec4 vC;in float vK;
void main(){vec2 q=gl_PointCoord*2.-1.;float r=dot(q,q);if(r>1.)discard;
 vec3 c=vC.rgb*(uSunC*.5*(.6+.4*(1.-r))+mix(uSkyH,uSkyZ,.5)*.6);oC=vec4(c*vC.a,vC.a*(1.-r*.5));}`;
// ---------------------------------------------------------------- post
export const FS_VS = `in vec3 aP;out vec2 vU;void main(){vU=aP.xy*.5+.5;gl_Position=vec4(aP.xy,0,1);}`;
export const FINAL_FS = `in vec2 vU;out vec4 oC;uniform sampler2D uC,uD;uniform float uAsp,uFoc,uAp,uNear,uFar,uBars,uFade,uGrain,uExp,uT,uHS,uSat,uVig,uLevels;uniform vec2 uShake;
float lin(float z){z=z*2.-1.;return 2.*uNear*uFar/(uFar+uNear-z*(uFar-uNear));}
vec3 aces(vec3 x){return clamp(x*(2.51*x+.03)/(x*(2.43*x+.59)+.14),0.,1.);}
void main(){vec2 u=vU;
 if(abs(u.y-.5)>.5-uBars){oC=vec4(0,0,0,1);return;}
 float z=lin(texture(uD,u).r);float coc=clamp(abs(z-uFoc)/z*uAp,0.,1.);
 vec3 c=textureLod(uC,u,0.).rgb;
 if(coc>.01){vec3 s=vec3(0);float W=0.;for(int i=0;i<16;i++){float a=float(i)*2.39996,r=sqrt((float(i)+.5)/16.);vec2 o=vec2(cos(a),sin(a))*r*coc*.022*vec2(1./uAsp,1.);
  float zz=lin(texture(uD,u+o).r);float cc=clamp(abs(zz-uFoc)/zz*uAp,0.,1.);float w=zz<z?max(cc,coc*.3):1.;
  s+=textureLod(uC,u+o,log2(1.+coc*uLevels*.022/4.)).rgb*w;W+=w;}c=mix(c,s/W,smoothstep(.01,.08,coc));}
 vec3 b=vec3(0);for(int i=2;i<7;i++){b+=textureLod(uC,u,float(i)).rgb*(1./float(i));}
 c+=b*.035;
 c*=uExp;
 if(uHS>0.){float l=dot(c,vec3(.3,.6,.1));c=mix(c,vec3(l)*vec3(.95,1.,1.08),.35*uHS);c*=1.+.25*uHS;}
 c=aces(c);
 float l=dot(c,vec3(.299,.587,.114));c=mix(vec3(l),c,uSat);
 c=pow(c,vec3(1./2.2));
 vec2 v=u-.5;c*=1.-dot(v,v)*uVig;
 c+=(fract(sin(dot(gl_FragCoord.xy+fract(uT)*91.,vec2(12.9898,78.233)))*43758.5453)-.5)*uGrain;
 c*=uFade;oC=vec4(c,1);}`;
