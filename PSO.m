function a = PSO(x,y,z,minRate,min1,min2,min3,xUser,yUser)
b = [100,2];
d = [100,2];
e = [1,2];
k = [1,100];
cu = minRate;
eta = 2.5;
b_0dB = -50;
b_0 = db2pow(b_0dB);
K_min = 4;  
K_max = 12;
A1 = db2pow(K_min);
A2 = (2/pi)*log((db2pow(K_max))/A1);
g = sqrt(1/2) + (randn(1,1) + 1i * randn(1,1));
    for i = 1:100
        b(i,1) = (0 - min1) + (min1 + min1) * rand() + x;
        b(i,2) = min2 + rand() * (min3 - min2);
        d(i,1) = b(i,1);
        d(i,2) = b(i,2);
        m = 0 - b(i,1) + x - min1;  
        n = x + min1 - b(i,1);
        c(i,1) = m + rand() * (n - m);
        m = min2 - b(i,2);
        n = min3 - b(i,2);
        c(i,2) = m + (n - m) * rand();
    end
    for i = 1:100
        for m = 1:3
        groundDisUAV_User(m) = sqrt((b(i,1)-xUser(m))^2 + (y-yUser(m))^2);
        DisUAV_User(m) = sqrt(groundDisUAV_User(m)^2 + b(i,2)^2);
        angleUAV_User(m) = atan(b(i,2)/groundDisUAV_User(m))*(180/pi);
        pow_LoS = b_0*(DisUAV_User(m)^(-eta));
        K_UAV_User(m) = A1*exp(A2*angleUAV_User(m)*(pi/180));
        g_UAV_User(m) = sqrt(K_UAV_User(m)/(1+K_UAV_User(m)))*g + sqrt(1/(1+K_UAV_User(m)))*g;
        h_UAV_Users(m) = sqrt(pow_LoS)*g_UAV_User(m);
        h_UAV_Users_dB(m) = pow2db(abs(h_UAV_Users(m))^2);
        end
        [pow_coef_array_ch(:), pow_coef_array_fr(:)] = findPowCoeff(abs(h_UAV_Users(:)),3);
        [achievableRate_ch(:), achievableRate_fr(:)] = findAchievableRate(h_UAV_Users(:),pow_coef_array_ch(:),pow_coef_array_fr(:),3);
        minR = min(achievableRate_ch(:));
        k(i) = minR;
        if(minR > cu)
            e(1) = b(i,1);
            e(2) = b(i,2);
            cu = minR;
        end
    end
    for i = 1:20
        for m = 1:100
            for n = 1:2
                c(m,n) = 0.8 * c(m,n) + rand() * 2.05 * (e(n) - c(m,n)) + rand() * 2.05 * (d(m,n) - c(m,n));
                b(m,n) = b(m,n) + c(m,n);
            end
                for s = 1:3
                    groundDisUAV_User(s) = sqrt((b(m,1)-xUser(s))^2 + (y-yUser(s))^2);
                    DisUAV_User(s) = sqrt(groundDisUAV_User(s)^2 + b(m,2)^2);
                    angleUAV_User(s) = atan(b(m,2)/groundDisUAV_User(s))*(180/pi);
                    pow_LoS = b_0*(DisUAV_User(s)^(-eta));
                    K_UAV_User(s) = A1*exp(A2*angleUAV_User(s)*(pi/180));
                    g_UAV_User(s) = sqrt(K_UAV_User(s)/(1+K_UAV_User(s)))*g + sqrt(1/(1+K_UAV_User(s)))*g;
                    h_UAV_Users(s) = sqrt(pow_LoS)*g_UAV_User(s);
                    h_UAV_Users_dB(s) = pow2db(abs(h_UAV_Users(s))^2);
                end
                [pow_coef_array_ch(:), pow_coef_array_fr(:)] = findPowCoeff(abs(h_UAV_Users(:)),3);
                [achievableRate_ch(:), achievableRate_fr(:)] = findAchievableRate(h_UAV_Users(:),pow_coef_array_ch(:),pow_coef_array_fr(:),3);
                minR = min(achievableRate_ch(:));
                if (minR > k(m))
                    k(m) = minR;
                    for l = 1:2
                        d(m,l) = b(m,l);
                    end
                end
                if(minR > cu)
                    e(1) = b(m,1);
                    e(2) = b(m,2);
                    cu = minR;
                end
         end
    end
    a(1) = e(1);
    a(2) = e(2);
              