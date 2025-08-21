Rw = 4.1;
Lw = 0.00215;
M = 1.541;
B = 1;

s = tf('s');

Ye = 1/(s*Lw + Rw);
Ym = 1/(M*s + B);
Km = 1/0.062;

Gp = Ye*Km*Ym/(1+(Ye*Km^2*Ym));

Ga = 3.27*16129032.26/(s+16129032.26);

Hs = 100/(s+100);

GHs = Gp * Ga * Hs;

mdp = 0.648929266709927;

ndp = 10 * mdp;

CF = 100;

Nf = 2.5023;

Hc = 100/(15.41 * s + 100);

Dp = ndp/((s+ndp)*s);

[Gm,Pm,Wcg,Wcp] = margin(1*GHs*Dp*Hc);
margin(1*GHs*Dp*Hc)

K0 = Gm;
wxo = Wcg;

G = Ga * Gp;
H = Hc * Hs;

wn = Wcg;
z = 1;
pmmax = 0;
wnmax = 0;
zmax = 0;

for n=1:100
    wn1 = wn + 0.05;
    wn2 = wn - 0.05;
    z1 = z + 0.01;
    z2 = z - 0.01;
    
    D1 = Dp*(s^2+2*z1*wn1*s+(wn1)^2)/(wn1^2);
    D2 = Dp*(s^2+2*z2*wn1*s+(wn1)^2)/(wn1^2);
    D3 = Dp*(s^2+2*z1*wn2*s+(wn2)^2)/(wn2^2);
    D4 = Dp*(s^2+2*z2*wn2*s+(wn2)^2)/(wn2^2);
    
    D5 = Dp*(s^2+2*z1*wn*s+(wn)^2)/(wn^2);
    D6 = Dp*(s^2+2*z2*wn*s+(wn)^2)/(wn^2);
    D7 = Dp*(s^2+2*z*wn1*s+(wn1)^2)/(wn1^2);
    D8 = Dp*(s^2+2*z*wn2*s+(wn2)^2)/(wn2^2);
    
    [Gm1,Pm1,Wcg1,Wcp1] = margin(Gm*D1*G*H);
    [Gm2,Pm2,Wcg2,Wcp2] = margin(Gm*D2*G*H);
    [Gm3,Pm3,Wcg3,Wcp3] = margin(Gm*D3*G*H);
    [Gm4,Pm4,Wcg4,Wcp4] = margin(Gm*D4*G*H);
    
    
    [Gm5,Pm5,Wcg5,Wcp5] = margin(Gm*D5*G*H);
    [Gm6,Pm6,Wcg6,Wcp6] = margin(Gm*D6*G*H);
    [Gm7,Pm7,Wcg7,Wcp7] = margin(Gm*D7*G*H);
    [Gm8,Pm8,Wcg8,Wcp8] = margin(Gm*D8*G*H);
    
    
    if(Pm1 > pmmax)
        pmmax = Pm1;
        wnmax = wn1;
        zmax = z1;
    end
    if(Pm2 > pmmax)
        pmmax = Pm2;
        wnmax = wn1;
        zmax = z2;
    end
    if(Pm3 > pmmax)
        pmmax = Pm3;
        wnmax = wn2;
        zmax = z1;
    end
    if(Pm4 > pmmax)
        pmmax = Pm4;
        wnmax = wn2;
        zmax = z2;
    end
    
    
    if(Pm5 > pmmax)
        pmmax = Pm5;
        wnmax = wn;
        zmax = z1;
    end
    if(Pm6 > pmmax)
        pmmax = Pm6;
        wnmax = wn;
        zmax = z2;
    end
    if(Pm7 > pmmax)
        pmmax = Pm7;
        wnmax = wn1;
        zmax = z;
    end
    if(Pm8 > pmmax)
        pmmax = Pm8;
        wnmax = wn2;
        zmax = z;
    end
    
    %{
    fprintf('Phase margin at %f, %f is %f\n', wn1,z1,Pm1 );
    fprintf('Phase margin at %f, %f is %f\n', wn1,z2,Pm2 );
    fprintf('Phase margin at %f, %f is %f\n', wn2,z1,Pm3 );
    fprintf('Phase margin at %f, %f is %f\n', wn2,z2,Pm4 );
    
    
    fprintf('Phase margin at %f, %f is %f\n', wn,z1,Pm5 );
    fprintf('Phase margin at %f, %f is %f\n', wn,z2,Pm6 );
    fprintf('Phase margin at %f, %f is %f\n', wn1,z,Pm7 );
    fprintf('Phase margin at %f, %f is %f\n', wn2,z,Pm8 );
    
    fprintf('\n');
    %}
    
    wn = wnmax;
    z = zmax;
end

za = transpose(zero(Dp*(s^2 + 2*z*wn*s + wn^2)/(wn^2)));
zb = transpose((zero(s^2 + 2*z*wn*s+wn^2)));

Dc = Dp * (s^2 + 2*zmax*wnmax*s + wnmax^2)/(wnmax^2);

K = 50;

[Gm,Pm,Wcg,Wcp] = margin(K*GHs*Dc*Hc);

z1 = za(1);
z2 = za(2);

Kp = K * ((1/-ndp) - (z1+z2)/(z1*z2));
Ki = K;
Kd = K * ((1/(-ndp)^2) - (z1 + z2 - (-ndp))/(-ndp*z1*z2));


