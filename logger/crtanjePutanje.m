footprint=load('robot_footprint.dat');

matDT=load('wh_dstar_dt.dat');
matDTdostupni=load('wh_dstar_dtdostupni.dat');
matPp=load('wh_dstar_pp.dat');
matpresao=load('wh_dstar_presao.dat');
matpresao7=load('wh_dstar_presao7.dat');
loudaj=0;
if loudaj==0
    WH_dstar_cost_map=load('wh_dstar_cost_map.dat');
else
    load punakarta
    load izracun %korakpokorak za punu kartu
%     load izracunobrnuto %korakpokorak za punu kartu
    matpresao=Pp;
end
if loudaj==0
WH_gridmap_x=load('wh_gridmap_x.dat');
	WH_gridmap_y=load('wh_gridmap_y.dat');
     WH_gridmap_static_cell=load('wh_gridmap_static_cell.dat');
else
    load punakartax
end
    metric=0.001;%metric=1;
    if loudaj==0
WH_gridmap_x=WH_gridmap_x*metric;
    WH_gridmap_y=WH_gridmap_y*metric;
    end
novex=[];novey=[];starex=[];starey=[];
for i=1:max(size(WH_gridmap_x))
                if(WH_gridmap_static_cell(i)==1)
                    starex=[starex;WH_gridmap_x(i)];
                    starey=[starey;WH_gridmap_y(i)];
               else
                    novex=[novex;WH_gridmap_x(i)];
                    novey=[novey;WH_gridmap_y(i)];
             end
end

% load cijelawit %unutra je WH_wit_f_cost od inicijalnog
% load cijelawitnova
%home za player-stage, prepisano iz data datoteke iz roota projekta 
origin=load('origin.dat');
Map_Home_x=origin(1);
Map_Home_y=origin(2);
xorigin=Map_Home_x;%player-stage
yorigin=Map_Home_y;
metric=0.001;
cell=load('cell_size.dat');
% cell=100;
% WH_planner_globalna_putanja_x=(load('wh_dstar_path_x.dat')-Map_Home_x)/cell;
% WH_planner_globalna_putanja_y=(load('wh_dstar_path_y.dat')-Map_Home_y)/cell;
WH_planner_globalna_putanja_x=(load('global_planner_path_x.dat')-Map_Home_x)/cell; %inic
WH_planner_globalna_putanja_y=(load('global_planner_path_y.dat')-Map_Home_y)/cell;
WH_globalna_putanja_x=(load('robot_globalna_putanja_x.dat')-Map_Home_x)/cell;%odvozena putanja
WH_globalna_putanja_y=(load('robot_globalna_putanja_y.dat')-Map_Home_y)/cell;
trenutna_planner_putanja_x=(load('global_planner_path_current_x.dat')-Map_Home_x)/cell;%sljedeci Pinitial
trenutna_planner_putanja_y=(load('global_planner_path_current_y.dat')-Map_Home_y)/cell;
Dstar_path_x=(load('path_tool_x.dat')-Map_Home_x)/cell;
Dstar_path_y=(load('path_tool_y.dat')-Map_Home_y)/cell;
Dstar_init_path_x=load('path_init_tool_x.dat')+0.5;
Dstar_init_path_y=load('path_init_tool_y.dat')+0.5;

pathpointeri=load('wh_svipathpointeri.dat');
poc=1;
prviput=1;
for i=1:max(size(pathpointeri)),
     if (pathpointeri(i)~=0)
 kraj=pathpointeri(i)+poc-1;
 if (i==max(size(pathpointeri))-1)
     prviput=0;
     prethodna_x=trenutna_planner_putanja_x(poc:kraj);
     prethodna_y=trenutna_planner_putanja_y(poc:kraj);
 end
 if (i==max(size(pathpointeri)))
     prviput=0;
     trenutna_x=trenutna_planner_putanja_x(poc:kraj);
     trenutna_y=trenutna_planner_putanja_y(poc:kraj);
 end
 poc=kraj+1;
 end
end
if prviput
    trenutna_x=WH_planner_globalna_putanja_x;
    trenutna_y=WH_planner_globalna_putanja_y;
end

trenutna_x=(load('path_robot_x.dat')-Map_Home_x)/cell;
trenutna_y=(load('path_robot_y.dat')-Map_Home_y)/cell;
mapocc=load('wh_gridmap_occupancy.dat');

cijena_prepreke=max(max(WH_dstar_cost_map));
% cijena_prepreke=2;

%parametri
crtanjeDT=0; %crtanje vrijednosti DTdostupni 0-bijelo 1-vrijednosti za bez surfa samo ukljuci crtajgridmapu=1
% matDTdostupni=matDT;
crtanjePp=0; %crtanje vrijednosti posjecenosti s plotom 0-ne 1-da
crtanjePpsurf=1; %crtanje vrijednosti posjecenosti surfom kao matrica DT 0-ne, 1-da (mora bit iskljuceno crtanjeDT)
crtanjepresao=0; %crtanje vrijednosti posjecenosti za vrijeme voznje robota - matpresao i trajektorija, mora bit 0 kad se ne crta
crtanjepresao7=0; %crtanje vrijednosti preklopa za vrijeme voznje robota - matpresao7 i trajektorija, mora bit ukljucen crtanjePpsurf
crtamale=0; %1-crtanje malih slika TSP1,2,skok1,2 - veliki fontovi
crtamasku=1; %1-trebalo za slike TSPskok1,2, kad se koristi P7 od korakpokorak.m
crtajgridmapu=0; %bez surfa samo plot
crtajoccupancy=0;
crtputanjice=1; %crtaj putanje za robot i tool
loadputanjice=0;%crtaj putanje izracunate u korakpokorak.m za robot i tool

% [sizex,sizey]=size(WH_dstar_cost_map);%tu mozes staviti DT
% matDT=mat_DT;%provjera sto je izracunao DTfinalnamu.m
% matPp=Pp; %provjera sto je izracunao DTfinalnamumudaljeodstarta.m
% matPp=P7; %provjera sto je izracunao korakpokorak.m TSP1,2,skok1,2 koriste ovo
% matDT=matDTdostupni;%provjera kak zgleda ovaj isti ok.
if (crtanjepresao7)
    matPp=matpresao7;
end
[sizex,sizey]=size(matDTdostupni);%tu mozes staviti DT
veca=zeros(sizex+10,sizey+10);
Xos=[0:1:sizex-1];
Yos=[0:1:sizey-1];
xv=[0:1:sizex+9];
yv=[0:1:sizey+9];


if crtanjePpsurf==1
redak=[10 11];
stupac=[10:1:25];
minf=1;
maxf=max(max(matPp));
for i=1:max(size(stupac)),
    matPp(redak(1),stupac(i))=minf+(maxf-minf)*(stupac(i)-stupac(1))/(stupac(max(size(stupac)))-stupac(1));
    matPp(redak(2),stupac(i))=minf+(maxf-minf)*(stupac(i)-stupac(1))/(stupac(max(size(stupac)))-stupac(1));
end
    matPp(redak(1),stupac(end)+1)=maxf;
    matPp(redak(2),stupac(end)+1)=maxf;
stupac=[10:1:26];
end

WH_gridmap_x=(load('wh_gridmap_x.dat')-Map_Home_x)/cell+1;%zbog indeksa, matlab ima indekse od 1
	WH_gridmap_y=(load('wh_gridmap_y.dat')-Map_Home_y)/cell+1;
for i=1:max(size(WH_gridmap_x))
    pomx=floor( WH_gridmap_x(i));
    pomy=floor( WH_gridmap_y(i));
    if pomx>=1 && pomx<=sizex && pomy>=1 && pomy<=sizey
%         if WH_gridmap_static_cell(i)==1
            WH_dstar_cost_map(pomx,pomy)=-1;
%         end
    end
end
figure
P=WH_dstar_cost_map-1;
maxP=max(max(P));
   for x=1:sizex,
       for y=1:sizey,
                  if (P(x,y)<maxP)
                    P(x,y)=0;
                  end
                  if (crtamasku==1)
                  if (P(x,y)==maxP)
                      if (matPp(x,y)==0)
                        matPp(x,y)=-2;
                      end
                      if (matpresao(x,y)<1)
                      matpresao(x,y)=-2;
                      end
                  end
                  end
	   if WH_dstar_cost_map(x,y)==-1
               matDTdostupni(x,y)=-1;
               if (matPp(x,y)==0)
                    matPp(x,y)=-1;
               end
               if (matpresao(x,y)==0)
               matpresao(x,y)=-1;
               end
               P(x,y)=2;
       end
%        if ((matPp(x,y)==0))
%            matDT(x,y)=-2;
%        end
       if (matPp(x,y)==0) && (matDT(x,y)>0)
           vici=1;
       end
       end
   end
   if crtanjeDT==1
    xint=WH_planner_globalna_putanja_x(1)+0.5
yint=WH_planner_globalna_putanja_y(1)+0.5
matDTdostupni(xint,yint)=1;
redak=[1:1:5];
stupac=[10:1:50];
minf=1;
maxf=max(max(matDTdostupni));
for i=1:max(size(stupac)),
    matDTdostupni(redak(1),stupac(i))=minf+(maxf-minf)*(stupac(i)-stupac(1))/(stupac(max(size(stupac)))-stupac(1));
    matDTdostupni(redak(2),stupac(i))=minf+(maxf-minf)*(stupac(i)-stupac(1))/(stupac(max(size(stupac)))-stupac(1));
end
end
veca(1:sizex,1:sizey)=matDTdostupni;
if (crtanjePpsurf==1)
    veca(1:sizex,1:sizey)=matPp;
end
if (crtanjepresao)
    veca(1:sizex,1:sizey)=matpresao;
end
% xmin=1; xmax=50; ymin=112; ymax=152;
xmin=1; xmax=sizex+1; ymin=1; ymax=sizey+1;
% xmin=55; xmax=120; ymin=80; ymax=105;
% xmin=1; xmax=40; ymin=82; ymax=115; %razminirac
% xmin=1; xmax=32; ymin=86; ymax=115;
%ovdje crtamo malu matricu karticu
if (1)
 osi=[xmin xmax ymin ymax];
kartica=veca;
 Xos=xv(osi(1):osi(2))+1;
Yos=yv(osi(3):osi(4))+1;
% Xos_real=Xos*cell+xorigin;
% Yos_real=Yos*cell+yorigin;
kartica=kartica(Xos,Yos);
Xos=Xos*cell+xorigin;
Yos=Yos*cell+yorigin;
brk=[];
   k=[1 0 0;1 1 1;0.8 0.8 0.8;0.6 0.6 0.6;0.4 0.4 0.4;0.3 0.3 0.3;0 0 0;brk];
k=[1 1 1;1 1 1;0.4 0.4 0.4;0.4 0.4 0.4;0.3 0.3 0.3;0 0 0;brk];

maxDT=max(max(matDTdostupni)); 
% k=hsv(maxDT);
k=hsv(maxDT+300);
k=k(300:end,:);

if (crtanjeDT==0)
    k=white(maxDT);
end
if (crtanjepresao)
    maxDT=max(max(matpresao))+5;
    k=hsv(maxDT);

end
colormap([0 0 0; 1 1 1;k]);

if (crtanjePpsurf==1)
        maxDT=max(max(matPp))+6;
    k=hsv(maxDT-1);
%     k(3,:)=[0.9 1 0.2];
    if (crtamasku==1)
%     colormap([0.55 0.55 0.55;0 0 0;1 1 1; 1 1 0;k]);
    colormap([0.75 0.75 0.75;0.75 0.75 0.75;1 1 1;k]);
    else
%     colormap([0 0 0;1 1 1;k]);
    colormap([0 0 0;1 1 1;0.75 0.75 0.75;k]);
    end
end
if(crtajgridmapu==0)
        if (crtamasku==1) && crtanjepresao
%     colormap([0.55 0.55 0.55;0 0 0;1 1 1; 1 1 0;k]);
    colormap([0.75 0.75 0.75;0 0 0;1 1 1;0 0 0;k]);
        end

% colormap([k]);
% pcolor(x,y,WH_dstar_f_cost_map','EdgeColor','none');
surf(Xos*metric,Yos*metric,kartica'*0,kartica','EdgeColor','none')%[0.7 0.7 0.7])
% surf(Xos_real*metric,Yos_real*metric,kartica'*0,kartica','EdgeColor',[0.5 0.5 0.5])%,'EdgeColor','none')
caxis([-1 maxDT])
if (crtamasku==1)
if (crtanjePpsurf==0) 
caxis([-2 maxDT])
else
    caxis([-2 maxDT])
end
end
grid off
% view([90 -90]);
end
grid off
view([0 90]);
% surf(xv*metric,yv*metric,veca'*0,veca','EdgeColor','none')
% caxis([minff-2 maxff])
end

hold on
% axis equal tight

plot(starex+cell*metric,starey+cell*metric,'k.');
plot(novex+cell*metric,novey+cell*metric,'k.');

if (crtajgridmapu) && (1 || (crtanjeDT)|| crtanjepresao)
   for x=1:1:sizex,
       for y=1:1:sizey,
           if (matDTdostupni(x,y)>0) && crtanjeDT
                               xc=(x+0.5)*cell+xorigin;
                    yc=(y+0.5)*cell+yorigin;
                        rucica=plot(xc*metric,yc*metric,'ks');
%                         pause
                        % text(xc*metric,yc*metric,num2str(mat_DTdostupni(k,l)),'FontSize',26)

                        set(rucica,'MarkerEdgeColor','None','MarkerFaceColor',k(mod((ceil(matDTdostupni(x,y))-1),maxDT)+1,:),'LineWidth',0.5);
%                         pause
           end
                      if (matpresao(x,y)>0) && crtanjepresao
                               xc=(x+0.5)*cell+xorigin;
                    yc=(y+0.5)*cell+yorigin;
                        rucica=plot(xc*metric,yc*metric,'ks');
%                         pause
                        % text(xc*metric,yc*metric,num2str(mat_DTdostupni(k,l)),'FontSize',26)

                        set(rucica,'MarkerEdgeColor','None','MarkerFaceColor',k(mod((ceil(matpresao(x,y))-1),maxDT)+1,:),'LineWidth',0.5);
%                         set(rucica,'MarkerEdgeColor','None','MarkerFaceColor',[0.7 1 0.5],'LineWidth',0.5);
%                         pause
           end
                     if (matpresao(x,y)<0) && crtanjepresao
                               xc=(x+0.5)*cell+xorigin;
                    yc=(y+0.5)*cell+yorigin;
                        rucica=plot(xc*metric,yc*metric,'ks');
%                         pause
                     end
                     if (matPp(x,y)) && crtanjePp
                               xc=(x+0.5)*cell+xorigin;
                    yc=(y+0.5)*cell+yorigin;
                        rucica=plot(xc*metric,yc*metric,'ys');
%                         pause
                     end
                    if (P(x,y)==2)
                               xc=(x+0.5)*cell+xorigin;
                    yc=(y+0.5)*cell+yorigin;
                        rucica=plot(xc*metric,yc*metric,'ks');
                    end
                    if crtajoccupancy && (mapocc(x,y)>50)
                              xc=(x+0.5)*cell+xorigin;
                    yc=(y+0.5)*cell+yorigin;
                        rucica=plot(xc*metric,yc*metric,'ks');
                        
                        set(rucica,'MarkerEdgeColor','None','MarkerFaceColor',k(mod((ceil(mapocc(x,y))-1),maxDT)+1,:),'LineWidth',0.5);
%                         k
                        pause
                    end

       end
   end

end

if (prviput) && ~isempty(WH_planner_globalna_putanja_x)
xint=WH_planner_globalna_putanja_x(1)+1
yint=WH_planner_globalna_putanja_y(1)+1
else
xint=trenutna_x(1)+1
yint=trenutna_y(1)+1
end
% kartica(xint,yint)
xc=xint*cell+xorigin;
yc=yint*cell+yorigin;
WH_globalna_putanja_x2=load('robot_globalna_putanja_x.dat');
WH_globalna_putanja_y2=load('robot_globalna_putanja_y.dat');
if ~isempty(WH_globalna_putanja_x2)
x_temp=(WH_globalna_putanja_x2(end)+cell)*metric;
y_temp=(WH_globalna_putanja_y2(end)+cell)*metric;
WH_globalna_putanja_th=load('robot_globalna_putanja_th.dat');
th_temp=WH_globalna_putanja_th(end);
footprint=footprint*metric;
    plot((x_temp+cos(th_temp)*footprint(:,1)-sin(th_temp)*footprint(:,2)),(y_temp+sin(th_temp)*footprint(:,1)+cos(th_temp)*footprint(:,2)),'k'); 
end
% plot(xc*metric,yc*metric,'ko')
% text(xc*metric,yc*metric-3,'S')
if (prviput) && ~isempty(WH_planner_globalna_putanja_x)
xint=WH_planner_globalna_putanja_x(end)+1
yint=WH_planner_globalna_putanja_y(end)+1
else
xint=trenutna_x(end)+1
yint=trenutna_y(end)+1
if (crtanjepresao) && ~isempty(WH_planner_globalna_putanja_x)
    if max(size(pathpointeri))>1
    xint=prethodna_x(end)+1
    yint=prethodna_y(end)+1
    xint=trenutna_x(end)+1
    yint=trenutna_y(end)+1
    else
xint=WH_planner_globalna_putanja_x(end)+1
yint=WH_planner_globalna_putanja_y(end)+1
    end
end
end
% kartica(xint,yint)
% xint=WH_globalna_putanja_x(end)+1
% yint=WH_globalna_putanja_y(end)+1

xc=xint*cell+xorigin;
yc=yint*cell+yorigin;
disp('cilj je')
xint
yint
if (crtanjeDT==0) && 0
plot(xc*metric,yc*metric,'ko')
text(xc*metric,yc*metric,'G')
end
if crtputanjice==1
    if (loadputanjice==1)
%     load putanjice
if 0
    plot(puttool(:,1),puttool(:,2),'k','LineWidth',1)
else
   
     plot(puttool(:,1),puttool(:,2),'b','LineWidth',1,'Color',[0 0 0.7])
%     plot(putrobot(:,1),putrobot(:,2),'y--','LineWidth',2,'Color',[1 0.3 0])
%     plot(putrobot(1,1),putrobot(1,2),'yo','LineWidth',1.5,'Color',[1 0.3 0])
%     text(putrobot(1,1),putrobot(1,2)-3,'S_R')
    plot(puttool(1,1),puttool(1,2),'bo','LineWidth',1.5,'Color',[0 0 0.7])
%     text(puttool(1,1),puttool(1,2)-3,'S_T')
%     plot(putrobot(end,1),putrobot(end,2),'yo','LineWidth',1.5,'Color',[1 0.3 0])
%     text(putrobot(end,1)-2,putrobot(end,2)+0.8,'G_R')
    plot(puttool(end,1),puttool(end,2),'bo','LineWidth',1.5,'Color',[0 0 0.7])
%     text(puttool(end,1)-2,puttool(end,2)+0.8,'G_T')
end
    else
        plot(((Dstar_init_path_x(1:end)+1)*cell+xorigin)*metric,((Dstar_init_path_y(1:end)+1)*cell+yorigin)*metric,'ko-','LineWidth',1.5,'Color',[0 0 0])
%         htool=plot(((Dstar_path_x(1:end)+1)*cell+xorigin)*metric,((Dstar_path_y(1:end)+1)*cell+yorigin)*metric,'bo-','LineWidth',1,'Color',[0 0 0.7])
%         hrobot=plot(((trenutna_x(1:end)+1)*cell+xorigin)*metric,((trenutna_y(1:end)+1)*cell+yorigin)*metric,'y*--','LineWidth',2,'Color',[1 0.3 0])
if (prviput)
%         plot(((trenutna_x(1)+1)*cell+xorigin)*metric,((trenutna_y(1)+1)*cell+yorigin)*metric,'yo','LineWidth',1.5,'Color',[1 0.3 0])
%         text(((trenutna_x(1)+1)*cell+xorigin)*metric,((trenutna_y(1)+1)*cell+yorigin)*metric-3,'S_R')
        plot(((Dstar_path_x(1)+1)*cell+xorigin)*metric,((Dstar_path_y(1)+1)*cell+yorigin)*metric,'bo','LineWidth',1.5,'Color',[0 0 0.7])
        text(((Dstar_path_x(1)+1)*cell+xorigin)*metric,((Dstar_path_y(1)+1)*cell+yorigin)*metric-3,'S_T')
end
%         plot(((trenutna_x(end)+1)*cell+xorigin)*metric,((trenutna_y(end)+1)*cell+yorigin)*metric,'yo','LineWidth',1.5,'Color',[1 0.3 0])
%         text(((trenutna_x(end)+1)*cell+xorigin)*metric-2,((trenutna_y(end)+1)*cell+yorigin)*metric+0.8,'G_R')
        plot(((Dstar_path_x(end)+1)*cell+xorigin)*metric,((Dstar_path_y(end)+1)*cell+yorigin)*metric,'bo','LineWidth',1.5,'Color',[0 0 0.7])
        text(((Dstar_path_x(end)+1)*cell+xorigin)*metric-2,((Dstar_path_y(end)+1)*cell+yorigin)*metric+0.8,'G_T')
% legend([htool,hrobot],'tool path','robot path')

    end
    
end
if (crtanjeDT==1)
    xint=redak(1)+3;
    yint=stupac(1)+1;
xc=xint*cell+xorigin;
yc=yint*cell+yorigin;    
text(xc*metric,yc*metric,'gmin')
    xint=redak(end)+2;
    yint=stupac(end);
xc=xint*cell+xorigin;
yc=yint*cell+yorigin;    
text(xc*metric,yc*metric,'gmax')
    xint=redak(end);
    yint=stupac(end)+3;
xc=xint*cell+xorigin;
yc=yint*cell+yorigin;    
text(xc*metric,yc*metric,'legend')
end
if (crtanjePpsurf==1)
    xint=redak(1)+3;
    yint=stupac(1)+2;
xc=xint*cell+xorigin;
yc=yint*cell+yorigin;    
text(xc*metric,yc*metric,'1st')
    xint=redak(1)+3;
    yint=stupac(end)-2;
xc=xint*cell+xorigin;
yc=yint*cell+yorigin;    
text(xc*metric,yc*metric,'3rd')
    xint=redak(end);
    yint=stupac(end)+3;
xc=xint*cell+xorigin;
yc=yint*cell+yorigin;    
text(xc*metric,yc*metric,'legend')
    xint=redak(end)+2;
    yint=stupac(6)+3;
xc=xint*cell+xorigin;
yc=yint*cell+yorigin;    
text(xc*metric,yc*metric,'2nd')
end

% xint=9+1.5
% yint=65+1.5
% kartica(xint,yint)
% xc=xint*cell+xorigin
% yc=yint*cell+yorigin
% plot(xc*metric,yc*metric,'ko')
% trenutna_x=puttool(:,1);
% trenutna_y=puttool(:,2);
duljina=0;
for i=2:length(Dstar_init_path_x)
        duljina=duljina+norm([Dstar_init_path_x(i) Dstar_init_path_y(i)]-[Dstar_init_path_x(i-1) Dstar_init_path_y(i-1)]);
end
% disp('put tool u m')
% duljina %u broju polja
% trenutna_x=putrobot(:,1);
% trenutna_y=putrobot(:,2);
% duljina=0;
% for i=2:length(trenutna_x)
%         duljina=duljina+norm([trenutna_x(i) trenutna_y(i)]-[trenutna_x(i-1) trenutna_y(i-1)]);
% end
% disp('put robot u m')
% duljina %u broju polja
if(0)
for i=1:length(WH_planner_globalna_putanja_x)
%    plot(((WH_planner_globalna_putanja_x(1:i)+1)*cell+xorigin)*metric,((WH_planner_globalna_putanja_y(1:i)+1)*cell+yorigin)*metric,'b-*','LineWidth',1.5)
   plot(((WH_planner_globalna_putanja_x(i)+1)*cell+xorigin)*metric,((WH_planner_globalna_putanja_y(i)+1)*cell+yorigin)*metric,'b-*','LineWidth',1.5)
%    text(((WH_planner_globalna_putanja_x(i)+1)*cell+xorigin)*metric,((WH_planner_globalna_putanja_y(i)+1)*cell+yorigin)*metric,5,mat2str(kartica(WH_planner_globalna_putanja_x(i)+0.5,WH_planner_globalna_putanja_y(i)+0.5)));

   pause
end 
for i=1:length(trenutna_x)
        plot(((trenutna_x(i)+1)*cell+xorigin)*metric,((trenutna_y(i)+1)*cell+yorigin)*metric,'k-*','LineWidth',1.5)

   pause
end 
else
    if (crtanjepresao)
        if max(size(pathpointeri))>1
%         plot(((prethodna_x+1)*cell+xorigin)*metric,((prethodna_y+1)*cell+yorigin)*metric,'b.-')%,'LineWidth',1.5)
        plot(((Dstar_path_x+1)*cell+xorigin)*metric,((Dstar_path_y+1)*cell+yorigin)*metric,'b','LineWidth',1,'Color',[0 0 0.7])%,'LineWidth',1.5)
%         plot(((trenutna_x+1)*cell+xorigin)*metric,((trenutna_y+1)*cell+yorigin)*metric,'y--','LineWidth',2,'Color',[1 0.3 0])%,'LineWidth',1.5)

        else
%     plot(((WH_planner_globalna_putanja_x+1)*cell+xorigin)*metric,((WH_planner_globalna_putanja_y+1)*cell+yorigin)*metric,'b.-')%,'LineWidth',2)%,'Color',[0.7 0.5 0.5])
        end
    else
        if crtanjeDT==0 && (crtputanjice==0)
%         plot(((trenutna_x+1)*cell+xorigin)*metric,((trenutna_y+1)*cell+yorigin)*metric,'r.-')%,'LineWidth',1.5)
%         plot(((Dstar_path_x+1)*cell+xorigin)*metric,((Dstar_path_y+1)*cell+yorigin)*metric,'b.-')%,'LineWidth',1.5)
        end
    end
end
if ((crtanjepresao) || (crtanjepresao7))  && ~isempty(WH_globalna_putanja_x)
    xint=WH_globalna_putanja_x(end)+1
yint=WH_globalna_putanja_y(end)+1
% kartica(xint,yint)
xc=xint*cell+xorigin;
yc=yint*cell+yorigin;
plot(xc*metric,yc*metric,'ko')
text(xc*metric,yc*metric,'R')
% plot(((trenutna_x+1)*cell+xorigin)*metric,((trenutna_y+1)*cell+yorigin)*metric,'m','LineWidth',1.5)

 plot(((WH_globalna_putanja_x+1)*cell+xorigin)*metric,((WH_globalna_putanja_y+1)*cell+yorigin)*metric,'k.-','Color',[0.542 0 0.446]);
 s=0;
path_x=((WH_globalna_putanja_x+1)*cell+xorigin)*metric;
path_y=((WH_globalna_putanja_y+1)*cell+yorigin)*metric;
for n=2:length(path_x)
    d2x=path_x(n);
    d2y=path_y(n);
    d1x=path_x(n-1);
    d1y=path_y(n-1);
    s=s+sqrt((d2x-d1x)^2+(d2y-d1y)^2);
end
disp('duljina trajektorije u m')

s
end
    %svi pointeri
    WH_dstar_x_reverse=(load('wh_dstar_x_reverse.dat')-Map_Home_x)/cell+1;
    WH_dstar_y_reverse=(load('wh_dstar_y_reverse.dat')-Map_Home_y)/cell+1;
    WH_dstar_dx_reverse=load('wh_dstar_dx_reverse.dat');
    WH_dstar_dy_reverse=load('wh_dstar_dy_reverse.dat');

%     quiver(WH_dstar_x_reverse*metric,WH_dstar_y_reverse*metric,WH_dstar_dx_reverse*metric,WH_dstar_dy_reverse*metric,0.2,'Color',[0.6 0.5 0.7]);


grid off
view([0 90]);
% naziv=strcat('funkcija f (min(f)=',num2str(minf),', max(f)=',num2str(maxf),')')
% title(naziv,'fontsize',16);

if (crtamale==1)
    ylabel('y [m]', 'fontsize',26,'fontname', 'times');
    xlabel('x [m]', 'fontsize',26,'fontname', 'times');
    h=gca;
    set(h,'fontsize',22,'fontname','times','box', 'on');
else
    ylabel('y [m]', 'fontsize',16,'fontname', 'times');
    xlabel('x [m]', 'fontsize',16,'fontname', 'times');
    h=gca;
    set(h,'fontsize',12,'fontname','times','box', 'on');
end
if (0) %ovo za naknadno mijenjanje figovima
    ylabel('y [m]', 'fontsize',22,'fontname', 'times');
    xlabel('x [m]', 'fontsize',22,'fontname', 'times');
    h=gca;
    set(h,'fontsize',18,'fontname','times','box', 'on');
end
if(0) %provjera posjecenosti
    for x=1:sizex,
       for y=1:sizey,
       if (matPp(x,y)==1)
   plot(((x+0.5)*cell+xorigin)*metric,((y+0.5)*cell+yorigin)*metric,'ys','Color',[0.6 0.6 0])
       end
       if (matPp(x,y)==-2)
   plot(((x+0.5)*cell+xorigin)*metric,((y+0.5)*cell+yorigin)*metric,'ys','Color',[1 1 1])
       end
       end
   end
    end
if (0) %debugiranje
    x=14+1;
    y=68+1;
       plot(((x+0.5)*cell+xorigin)*metric,((y+0.5)*cell+yorigin)*metric,'ks')
end
    maxboja=7;
boja=hsv(maxboja);
   
%kad hocu izbrojati one koje je presao:

if (crtanjepresao)
matPp=matpresao; %zakomentiraj kad hoces izbrojati od puta redundantne a ne od trajektorije
end
% matPp=load('wh_dstar_pp.dat');
% matPp=load('wh_dstar_presao.dat');

    maxposj=max(max(matPp));
brojponovnih=zeros(1,maxposj);
for x=1:sizex
      for y=1:sizey
 for k=1:maxposj         
          if (matPp(x,y)==k)
brojponovnih(k)=brojponovnih(k)+1;
if (crtanjePp==1)
   rucica=plot(((x+0.5)*cell+xorigin)*metric,((y+0.5)*cell+yorigin)*metric,'ys');
set(rucica,'Color',boja(mod((k-1),maxboja)+1,:));
end
          end
 end
      end
end
 
disp('brojponovnih 1 put 2 put 3 put itd')
    brojponovnih           

disp('duljina puta u m')
duljina*cell*metric
disp('ukupan broj za posjetiti u m^2')
ukupan=brojponovnih*ones(maxposj,1)*cell*metric*cell*metric

disp('redundantno posjecivanje u m^2')
redund=0;
for k=2:maxposj
    redund=redund+(k-1)*brojponovnih(k);
end
redund=redund*cell*metric*cell*metric


axis equal tight
xmin=xmin*cell+xorigin;
xmax=xmax*cell+xorigin;
ymin=ymin*cell+yorigin;
ymax=ymax*cell+yorigin;
box off
axis([xmin xmax ymin ymax]*metric);
% axis([0.1 2.6 -2.6 -0.1]) %TSP1
% axis([0.8 3.3 -2.6 -0.1]) %TSP2
% axis([-0.1 2.4 0.8 3.3]) %TSPskok1
% axis([-0.1 2.4 1.3 3.8]) %TSPskok2
