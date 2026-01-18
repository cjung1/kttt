function a = jDE(x,y,z,minRate,min1,min2,min3,xUser,yUser)
b = [20,2];
e = [1,2];
spu = minRate;
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
    h = rand();
    if(h < 0.1)
        F = F * h + (1 - h) * (1 - F);
    end
    h = rand();
    if(h < 0.1)
        CR = CR * h + (1 - h) * (1 - CR);
    end
    for r = 1:2
        p = rand();
        if(p <= CR || d == r)
            if(b(n,r) - b(j,r) < 0)
                f(r) = b(m,r) + F * (b(j,r) - b(n,r));
            else
                f(r) = b(m,r) + F * (b(n,r) - b(j,r));
            end
        else 
            f(r) = e(r);
        end
    end
        for j = 1:3
            groundDisUAV_User(j) = sqrt((f(1)-xUser(j))^2 + (y-yUser(j))^2);
            DisUAV_User(j) = sqrt(groundDisUAV_User(j)^2 + f(2)^2);
            angleUAV_User(j) = atan(f(2)/groundDisUAV_User(j))*(180/pi);
            pow_LoS = b_0*(DisUAV_User(j)^(-eta));
            K_UAV_User(j) = A1*exp(A2*angleUAV_User(j)*(pi/180));
            g_UAV_User(j) = sqrt(K_UAV_User(j)/(1+K_UAV_User(j)))*g + sqrt(1/(1+K_UAV_User(j)))*g;
            h_UAV_Users(j) = sqrt(pow_LoS)*g_UAV_User(j);
            h_UAV_Users_dB(j) = pow2db(abs(h_UAV_Users(j))^2);
        end
            [pow_coef_array_ch(:), pow_coef_array_fr(:)] = findPowCoeff(abs(h_UAV_Users(:)),3);
            [achievableRate_ch(:), achievableRate_fr(:)] = findAchievableRate(h_UAV_Users(:),pow_coef_array_ch(:),pow_coef_array_fr(:),3);
            mino = min(achievableRate_ch(:));
        if (mino > spu)
            spu = mino;
            for l = 1:2
                e(l) = f(l);
            end
        end
end
a(1) = e(1);
a(2) = e(2);

