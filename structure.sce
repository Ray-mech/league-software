//init
    clear;
    clf;
    
//define function
    //positive definite?
    function [ispos] = isPos(X)
        ev = spec(X);
        ispos = "POSITIVE";
        for i = ev',
            if real(i) < 0.0 then
                ispos = "NON-POSITIVE";
            end,
        end
    endfunction
    
//parameters of the structure
    //rigid bodies
    m11=15;  m12=20; m21=20; m22=15;
    j11=7.5; j12=10; j21=10; j22=7.5;

    //links
    dv11=0.3; dv12=0.3; dv13=0.3; dv21=0.3; dv22=0.3; dv23=0.3;
    kv11=10000; kv12=10000; kv13=10000; kv21=10000; kv22=10000; kv23=10000;
    
    dc1=0.4; dc2=0.4; dc3=0.4; dc4=0.4; dc5=0.4; dc6=0.4;
    kc1=11000; kc2=11000; kc3=11000; kc4=11000; kc5=11000; kc6=11000;
    
    //connection
    l111=1; l112=1; l113=1; l114=1; l121=1; l122=1; l123=1; l124=1;
    l211=1; l212=1; l213=1; l214=1; l221=1; l222=1; l223=1; l224=1;
    psi111=%pi/3; psi112=%pi/3; psi113=%pi/3; psi114=%pi/3;
    psi121=%pi/3; psi122=%pi/3; psi123=%pi/3; psi124=%pi/3;
    psi211=%pi/3; psi212=%pi/3; psi213=%pi/3; psi214=%pi/3;
    psi221=%pi/3; psi222=%pi/3; psi223=%pi/3; psi224=%pi/3;
    phi111=%pi/3; phi112=%pi/3; phi113=%pi/3; phi114=%pi/3;
    phi121=%pi/3; phi122=%pi/3; phi123=%pi/3; phi124=%pi/3;
    phi211=%pi/3; phi212=%pi/3; phi213=%pi/3; phi214=%pi/3;
    phi221=%pi/3; phi222=%pi/3; phi223=%pi/3; phi224=%pi/3;
    
// parameter of controller
 //**select controller**//
    slctCtrl = 2
 //*********************//
    select slctCtrl,
        case 1 then
            //P,Q: POSITIVE definite
            z1=17; p1=10; k1=1; alp1=1; bet1=100;  gam1=15000;
            z2=17; p2=10; k2=1; alp2=4; bet2=400;  gam2=7500;,
        case 2 then
            //P,Q: NON-POSITIVE definite
            z1=17; p1=5; k1=1; alp1=1; bet1=100;  gam1=400;
            z2=17; p2=5; k2=1; alp2=2; bet2=400;  gam2=100;,
        else disp("invalid controller select")
    end
    
//matrices of subsystem1,2
    M1 = diag([m11,m11,j11,m12,m12,j12]);
    M2 = diag([m21,m21,j21,m22,m22,j22]);
    
    N11 = [0 0 -cos(phi111); 1 1 sin(phi111); -l111*cos(psi111) l112*cos(psi112) -l111*sin(psi111+phi111)];
    N12 = [0 0 -cos(phi122); 1 1 sin(phi122); -l121*cos(psi121) l122*cos(psi122) -l122*sin(psi122+phi122)];
    N21 = [0 0 -cos(phi211); 1 1 sin(phi211); -l211*cos(psi211) l212*cos(psi212) -l211*sin(psi211+phi211)];
    N22 = [0 0 -cos(phi222); 1 1 sin(phi222); -l221*cos(psi221) l222*cos(psi222) -l222*sin(psi222+phi222)];

    D1 = [N11; -N12] * diag([dv11 dv12 dv13]) * [N11' -N12'];
    D2 = [N21; -N22] * diag([dv21 dv22 dv23]) * [N21' -N22'];
    
    K1 = [N11; -N12] * diag([kv11 kv12 kv13]) * [N11' -N12'];
    K2 = [N21; -N22] * diag([kv21 kv22 kv23]) * [N21' -N22'];

    L1 = sysdiag(eye(3,3),zeros(3,3));
    L2 = sysdiag(zeros(3,3),eye(3,3));

//matrices of overall system
    Nb121 = [1 1 sin(phi114);
             0 0 cos(phi114);
             l114*cos(psi113) -l113*cos(psi114) l114*sin(psi114+phi114)];
    Nb122 = [1 1 sin(phi124);
             0 0 cos(phi124);
             l124*cos(psi124) -l123*cos(psi124) l124*sin(psi124+phi124)];
    Nb12 = sysdiag(Nb121,Nb122);
            
    Nb211 = [1 1 sin(phi213);
             0 0 cos(phi213);
             l214*cos(psi213) -l213*cos(psi214) -l213*sin(psi213+phi213)];
    Nb212 = [1 1 sin(phi223);
             0 0 cos(phi223);
             l224*cos(psi224) -l223*cos(psi224) -l223*sin(psi223+phi223)];
    Nb21 = sysdiag(Nb211,Nb212);

    Mb = sysdiag(M1,M2);    
    Db = sysdiag(D1,D2) + [Nb12; -Nb21]*diag([dc1 dc2 dc3 dc4 dc5 dc6])*[Nb12' -Nb21'];
    Kb = sysdiag(K1,K2) + [Nb12; -Nb21]*diag([kc1 kc2 kc3 kc4 kc5 kc6])*[Nb12' -Nb21'];
    Lb = sysdiag(L1,L2);
    
//matrices of overall controller
    zb = sysdiag(z1*eye(6,6),z2*eye(6,6));
    pb = sysdiag(p1*eye(6,6),p2*eye(6,6));
    kb = sysdiag(k1*eye(6,6),k2*eye(6,6));
    alpb=sysdiag(alp1*eye(6,6),alp2*eye(6,6));
    betb=sysdiag(bet1*eye(6,6),bet2*eye(6,6));
    gamb=sysdiag(gam1*eye(6,6),gam2*eye(6,6));

//matrices of overall system with controller
    Ab = [zeros(12,12) eye(12,12) zeros(12,12) zeros(12,12);
          -inv(Mb)*Kb -inv(Mb)*Db kb*pb*inv(Mb)*Lb kb*pb/zb*inv(Mb)*Lb;
          zeros(12,12) zeros(12,12) zeros(12,12) eye(12,12);
          zeros(12,12) zeros(12,12) zeros(12,12) zeros(12,12)];
    Bb = [zeros(12,12); zeros(12,12); zeros(12,12); eye(12,12)];

//controller
    R1 = eye(6,6);
    R2 = eye(6,6);
    Rh = sysdiag(R1,R2);
    Rb = inv(Rh);

    Pb11 = 1/2*(gamb^2*zb*alpb*inv(kb)*inv(pb)*Db+Db*zb*alpb*inv(kb)*inv(pb)*gamb^2)+1/2*(Kb*zb*betb*inv(kb)*inv(pb)*gamb^2+gamb^2*zb*betb*inv(kb)*inv(pb)*Kb)+gamb^2*alpb*betb*Lb*Rh*Lb';
    Pb12 = inv(kb)*inv(pb)*zb*gamb^2*alpb*Mb;
    Pb13 = zeros(12,12);
    Pb14 = Lb*alpb*gamb;
    Pb22 = inv(kb)*inv(pb)*zb*gamb^2*betb*Mb;
    Pb23 = zeros(12,12);
    Pb24 = Lb*betb*gamb;
    Pb33 = gamb*pb*(gamb+pb)*Rb-Lb'*inv(Mb)*Lb*kb*pb*betb*gamb;
    Pb34 = gamb*pb*Rh;
    Pb44 = (gamb+pb)*Rb;
    Pb = [Pb11  Pb12  Pb13  Pb14;
          Pb12' Pb22  Pb23  Pb24;
          Pb13' Pb23' Pb33  Pb34;
          Pb14' Pb24' Pb34' Pb44];
    disp(strcat(["Pb:",isPos(Pb)]));
    
    Qb11 = gamb^2*alpb^2*Lb*Rh*Lb'+kb^(-1)*pb^(-1)*zb*gamb^2*alpb*Kb+Kb*alpb*gamb^2*zb*pb^(-1)*kb^(-1);
    Qb12 = 1/2*(kb^(-1)*pb^(-1)*zb*gamb^2*alpb*Db-Db*alpb*gamb^2*zb*pb^(-1)*kb^(-1))+1/2*(Kb*kb^(-1)*pb^(-1)*zb*gamb^2*betb-betb*gamb^2*zb*pb^(-1)*kb^(-1)*Kb);
    Qb13 = Lb*(pb-zb)*alpb*gamb^2;
    Qb14 = Kb*inv(Mb)*Lb*betb*gamb+Lb*alpb*gamb*pb;
    Qb22 = gamb^2*betb^2*Lb*Lb'-kb^(-1)*pb^(-1)*zb*gamb^2*alpb*Mb+kb^(-1)*pb^(-1)*zb*gamb^2*betb*Db+Db*betb*gamb^2*zb*pb^(-1)*kb^(-1)-Mb*alpb*gamb^2*zb*pb^(-1)*kb^(-1);
    Qb23 = Lb*(pb-zb)*betb*gamb^2;
    Qb24 = Lb*(betb*pb-gamb*alpb)+Db*inv(Mb)*Lb*betb*gamb;
    Qb33 = gamb^2*pb^2*Rb;
    Qb34 = zeros(12,12);
    Qb44 = -2*gamb*betb*kb*pb*zb^(-1)*Lb'*Mb*Lb+(gamb^2+pb^2)*Rb;
    Qb = [Qb11  Qb12  Qb13  Qb14;
          Qb12' Qb22  Qb23  Qb24;
          Qb13' Qb23' Qb33  Qb34;
          Qb14' Qb24' Qb34' Qb44];
    disp(strcat(["Qb:",isPos(Qb)]));
    
    //Feedback gain F
    F = inv(Rb)*Bb'*Pb;
    
//simulation
    Cb = sysdiag(eye(12,12),zeros(12,12),zeros(24,24));
    G = syslin('c',Ab-Bb*F,Bb,Cb);
    
    tEnd = 200;
    tStep = 0.01;
    t = 0:tStep:tEnd;
    r = zeros(12,tEnd/tStep+1);
    x0 = [0.2 -0.2 0.5 zeros(1,9) zeros(1,36)]'; 
    y = csim(r,t,G,x0);
    
//draw
    //pause;
    subplot(2,2,1);
      plot(t',y(1:3,:)'); xlabel("time,s");
      xgrid(4); xtitle("rigid body 11");
    subplot(2,2,2);
      plot(t',y(7:9,:)'); xlabel("time,s");
      xgrid(4); xtitle("rigid body 21");
    subplot(2,2,3);
      plot(t',y(4:6,:)');
      xgrid(4); xtitle("rigid body 12");
    subplot(2,2,4);
      plot(t',y(10:12,:)');
      xgrid(4); xtitle("rigid body 22");
      //legends(['$x_{ij}(t)$','$y_{ij}(t)$','$\theta_{ij}(t)$'],[2,13,5],font_size=3);
      
//writeCSV (for visualize software)
    filepath = "D:\resData.csv"
    //write_csv(y',filepath);
