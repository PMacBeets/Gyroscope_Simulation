function xdot = eom3(t,x)

    a = x(1);
    b = x(2);
    y = x(3);
    d = x(4);
    a_d = x(5);
    b_d = x(6);
    y_d = x(7);
    d_d = x(8);
    
    %alpha_dd_num = ;
    %alpha_dd_den = ;
    a_dd = (1688849860263936*cos(y)*(((18015*pi)/2 + 66825912621352060825/562949953421312)*(b_d*y_d*cos(y) - a_d*b_d*cos(b)*cos(y) + a_d*y_d*sin(b)*sin(y)) - 7350*sin(b)*sin(y) + (825*b_d*d_d*cos(y))/2 + 900*b_d*y_d*cos(y) + (y_d + a_d*cos(b))*(b_d*cos(y) + a_d*sin(b)*sin(y))*(15*pi + 16821072580431790825/562949953421312) - ((18015*pi)/2 + 30627964525063665375/281474976710656)*(y_d + a_d*cos(b))*(b_d*cos(y) + a_d*sin(b)*sin(y)) - sin(b)*sin(y)*(4410*pi + 2041864301670911025/35184372088832) - (21675*a_d^2*cos(b)*sin(b)*sin(y))/2 - 22575*a_d*b_d*cos(b)*cos(y) + (825*a_d*d_d*sin(b)*sin(y))/2 + 900*a_d*y_d*sin(b)*sin(y)))/(sin(b)*(15212315116327403520*pi + 220300613098904131275)) + (844424930131968*sin(y)*(7350*sin(b)*cos(y) - ((18015*pi)/2 + 30627964525063665375/281474976710656)*(a_d*b_d*cos(b)*sin(y) - b_d*y_d*sin(y) + a_d*y_d*sin(b)*cos(y)) + (825*b_d*d_d*sin(y))/2 + 900*b_d*y_d*sin(y) + (y_d + a_d*cos(b))*(b_d*sin(y) - a_d*sin(b)*cos(y))*(15*pi + 16821072580431790825/562949953421312) - (y_d + a_d*cos(b))*((18015*pi)/2 + 66825912621352060825/562949953421312)*(b_d*sin(y) - a_d*sin(b)*cos(y)) + sin(b)*cos(y)*(4410*pi + 2041864301670911025/35184372088832) + (21675*a_d^2*cos(b)*sin(b)*cos(y))/2 - 22575*a_d*b_d*cos(b)*sin(y) - (825*a_d*d_d*sin(b)*cos(y))/2 - 900*a_d*y_d*sin(b)*cos(y)))/(sin(b)*(7606157558163701760*pi + 101795331192614970525));


    %beta_dd_num = ;
    %beta_dd_den = ;
    b_dd = (844424930131968*cos(y)*(7350*sin(b)*cos(y) - ((18015*pi)/2 + 30627964525063665375/281474976710656)*(a_d*b_d*cos(b)*sin(y) - b_d*y_d*sin(y) + a_d*y_d*sin(b)*cos(y)) + (825*b_d*d_d*sin(y))/2 + 900*b_d*y_d*sin(y) + (y_d + a_d*cos(b))*(b_d*sin(y) - a_d*sin(b)*cos(y))*(15*pi + 16821072580431790825/562949953421312) - (y_d + a_d*cos(b))*((18015*pi)/2 + 66825912621352060825/562949953421312)*(b_d*sin(y) - a_d*sin(b)*cos(y)) + sin(b)*cos(y)*(4410*pi + 2041864301670911025/35184372088832) + (21675*a_d^2*cos(b)*sin(b)*cos(y))/2 - 22575*a_d*b_d*cos(b)*sin(y) - (825*a_d*d_d*sin(b)*cos(y))/2 - 900*a_d*y_d*sin(b)*cos(y)))/(7606157558163701760*pi + 101795331192614970525) - (1688849860263936*sin(y)*(((18015*pi)/2 + 66825912621352060825/562949953421312)*(b_d*y_d*cos(y) - a_d*b_d*cos(b)*cos(y) + a_d*y_d*sin(b)*sin(y)) - 7350*sin(b)*sin(y) + (825*b_d*d_d*cos(y))/2 + 900*b_d*y_d*cos(y) + (y_d + a_d*cos(b))*(b_d*cos(y) + a_d*sin(b)*sin(y))*(15*pi + 16821072580431790825/562949953421312) - ((18015*pi)/2 + 30627964525063665375/281474976710656)*(y_d + a_d*cos(b))*(b_d*cos(y) + a_d*sin(b)*sin(y)) - sin(b)*sin(y)*(4410*pi + 2041864301670911025/35184372088832) - (21675*a_d^2*cos(b)*sin(b)*sin(y))/2 - 22575*a_d*b_d*cos(b)*cos(y) + (825*a_d*d_d*sin(b)*sin(y))/2 + 900*a_d*y_d*sin(b)*sin(y)))/(15212315116327403520*pi + 220300613098904131275)


    %gamma_dd_num = ;
    %gamma_dd_den = ;
    y_dd = (1113996714244946015*b_d^2*sin(2*y) - 1113996714244946015*a_d^2*sin(2*y) + 8956422460662608360*a_d*b_d*sin(b) + 2227993428489892030*a_d^2*cos(b)^2*cos(y)*sin(y) - 4455986856979784060*a_d*b_d*sin(b)*cos(y)^2 + 3377699720527872*pi*a_d*b_d*sin(b))/(3377699720527872*pi + 6728429032172716330) - (1688849860263936*cos(b)*cos(y)*(((18015*pi)/2 + 66825912621352060825/562949953421312)*(b_d*y_d*cos(y) - a_d*b_d*cos(b)*cos(y) + a_d*y_d*sin(b)*sin(y)) - 7350*sin(b)*sin(y) + (825*b_d*d_d*cos(y))/2 + 900*b_d*y_d*cos(y) + (y_d + a_d*cos(b))*(b_d*cos(y) + a_d*sin(b)*sin(y))*(15*pi + 16821072580431790825/562949953421312) - ((18015*pi)/2 + 30627964525063665375/281474976710656)*(y_d + a_d*cos(b))*(b_d*cos(y) + a_d*sin(b)*sin(y)) - sin(b)*sin(y)*(4410*pi + 2041864301670911025/35184372088832) - (21675*a_d^2*cos(b)*sin(b)*sin(y))/2 - 22575*a_d*b_d*cos(b)*cos(y) + (825*a_d*d_d*sin(b)*sin(y))/2 + 900*a_d*y_d*sin(b)*sin(y)))/(sin(b)*(15212315116327403520*pi + 220300613098904131275)) - (844424930131968*cos(b)*sin(y)*(7350*sin(b)*cos(y) - ((18015*pi)/2 + 30627964525063665375/281474976710656)*(a_d*b_d*cos(b)*sin(y) - b_d*y_d*sin(y) + a_d*y_d*sin(b)*cos(y)) + (825*b_d*d_d*sin(y))/2 + 900*b_d*y_d*sin(y) + (y_d + a_d*cos(b))*(b_d*sin(y) - a_d*sin(b)*cos(y))*(15*pi + 16821072580431790825/562949953421312) - (y_d + a_d*cos(b))*((18015*pi)/2 + 66825912621352060825/562949953421312)*(b_d*sin(y) - a_d*sin(b)*cos(y)) + sin(b)*cos(y)*(4410*pi + 2041864301670911025/35184372088832) + (21675*a_d^2*cos(b)*sin(b)*cos(y))/2 - 22575*a_d*b_d*cos(b)*sin(y) - (825*a_d*d_d*sin(b)*cos(y))/2 - 900*a_d*y_d*sin(b)*cos(y)))/(sin(b)*(7606157558163701760*pi + 101795331192614970525));



    %delta_dd_num = ;
    %delta_dd_den = ;
    d_dd = -(1113996714244946015*((b_d^2*sin(2*y))/2 - a_d^2*sin(b)^2*cos(y)*sin(y) - a_d*b_d*sin(b)*cos(y)^2 + a_d*b_d*sin(b)*sin(y)^2))/(1688849860263936*pi + 3364214516086358165);

   
    xdot = [a_d;b_d;y_d;d_d;a_dd;b_dd;y_dd;d_dd];
    
    
end
