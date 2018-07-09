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
                ispos = "NOT POSITIVE";
            end,
        end
    endfunction

for i=[2:10],
for j=[3:5000],
for k=[1:20000],

//parameters of the structure
    //rigid bodies
    m11=10; m12=10; m21=10; m22=10;
    j11=50; j12=50; j21=50; j22=50;

    //links
    dv11=1;   dv12=1;   dv13=1;   dv21=1;   dv22=1;   dv23=1;
    kv11=200; kv12=200; kv13=200; kv21=200; kv22=200; kv23=200;
    
    dc1=0.8; dc2=0.8; dc3=0.8; dc4=0.8; dc5=0.8; dc6=0.8;
    kc1=500; kc2=500; kc3=500; kc4=500; kc5=500; kc6=500;
    
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
    z1=15; p1=10; k1=1; alp1=1; bet1=50;  gam1=7000;
    z2=15; p2=10; k2=1; alp2=i; bet2=j; gam2=k;
    
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
    
disp(strcat([string(i) " " string(j) " " string(k)]))
if isPos(Qb)=="POSITIVE" & isPos(Pb)=="POSITIVE" then
    disp(strcat(["Pb:",isPos(Pb)]));
    disp(strcat(["Qb:",isPos(Qb)]));
    disp(i);
    pause;
end,

end,end,end
