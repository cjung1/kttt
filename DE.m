function a = DE(x,y,z,minRate,min1,min2,min3,xUser,yUser)
b = [20,2];
e = [1,2];
su = minRate;
eta = 2.5;
b_0dB = -50;
b_0 = db2pow(b_0dB);
K_min = 4;  
K_max = 12;
A1 = db2pow(K_min);
A2 = (2/pi)*log((db2pow(K_max))/A1);
g = sqrt(1/2) + (randn(1,1) + 1i * randn(1,1));
e(1) = (x - min1) + rand() * 2 * min1;
e(2) = min2 + rand() * (min3 - min2);
for i = 1:3
groundDisUAV_User(i) = sqrt((e(1)-xUser(i))^2 + (y-yUser(i))^2);
DisUAV_User(i) = sqrt(groundDisUAV_User(i)^2 + e(2)^2);
angleUAV_User(i) = atan(e(2)/groundDisUAV_User(i))*(180/pi);
pow_LoS = b_0*(DisUAV_User(i)^(-eta));
K_UAV_User(i) = A1*exp(A2*angleUAV_User(i)*(pi/180));
g_UAV_User(i) = sqrt(K_UAV_User(i)/(1+K_UAV_User(i)))*g + sqrt(1/(1+K_UAV_User(i)))*g;
h_UAV_Users(i) = sqrt(pow_LoS)*g_UAV_User(i);
h_UAV_Users_dB(i) = pow2db(abs(h_UAV_Users(i))^2);
end
[pow_coef_array_ch(:), pow_coef_array_fr(:)] = findPowCoeff(abs(h_UAV_Users(:)),3);
[achievableRate_ch(:), achievableRate_fr(:)] = findAchievableRate(h_UAV_Users(:),pow_coef_array_ch(:),pow_coef_array_fr(:),3);
mina = min(achievableRate_ch(:));
F = 0.9;
CR = 0.8;
for i = 1:20
    b(i,1) = (x - min1) + (2 * min1) * rand();
    b(i,2) = min2 + (min3 - min2) * rand();
end
for i = 1:400
    m = randi([1,20]);
    n = randi([1,20]);
    j = randi([1,20]);
    d = randi([1,2]);
    for r = 1:2
        p = rand();
        if(p <= CR || d == r)
            f(r) = b(m,r) + F * (b(n,r) - b(j,r));
        else 
            f(r) = e(r);
        end
    end
        for r = 1:3
            groundDisUAV_User(r) = sqrt((f(1)-xUser(r))^2 + (y-yUser(r))^2);
            DisUAV_User(r) = sqrt(groundDisUAV_User(r)^2 + f(2)^2);
            angleUAV_User(r) = atan(f(2)/groundDisUAV_User(r))*(180/pi);
            pow_LoS = b_0*(DisUAV_User(r)^(-eta));
            K_UAV_User(r) = A1*exp(A2*angleUAV_User(r)*(pi/180));
            g_UAV_User(r) = sqrt(K_UAV_User(r)/(1+K_UAV_User(r)))*g + sqrt(1/(1+K_UAV_User(r)))*g;
            h_UAV_Users(r) = sqrt(pow_LoS)*g_UAV_User(r);
            h_UAV_Users_dB(r) = pow2db(abs(h_UAV_Users(r))^2);
        end
            [pow_coef_array_ch(:), pow_coef_array_fr(:)] = findPowCoeff(abs(h_UAV_Users(:)),3);
            [achievableRate_ch(:), achievableRate_fr(:)] = findAchievableRate(h_UAV_Users(:),pow_coef_array_ch(:),pow_coef_array_fr(:),3);
            mina = min(achievableRate_ch(:));
        if (mina > su)
            su = mina;
            for l = 1:2
                e(l) = f(l);
            end
        end
end
a(1) = e(1);
a(2) = e(2);

