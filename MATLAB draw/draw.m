data=load("D:\桌面\WHU\senior\study\组合导航\大作业\2022组合导航课程考核与数据\result.txt");
refdata=load("D:\桌面\WHU\senior\study\组合导航\大作业\2022组合导航课程考核与数据\参考数据.txt");
t=data(:,1);rt=refdata(:,2);
b=data(:,2);rb=refdata(:,3);
l=data(:,3);rl=refdata(:,4);
h=data(:,4);rh=refdata(:,5);
vn=data(:,5);rvn=refdata(:,6);
ve=data(:,6);rve=refdata(:,7);
vd=data(:,7);rvd=refdata(:,8);
roll=data(:,8);rroll=refdata(:,9);
pitch=data(:,9);rpitch=refdata(:,10);
yaw=data(:,10);ryaw=refdata(:,11);

figure(1);
subplot(3,1,1);
hold on;
%plot(t,b);plot(rt,rb);legend("my","ref");
plot(t(1:size(rt,1)),b(1:size(rt,1))-rb);
hold off;
title("latitude(B)");
subplot(3,1,2);
hold on;
%plot(t,l);plot(rt,rl);legend("my","ref");
plot(t(1:size(rt,1)),l(1:size(rt,1))-rl);
hold off;
title("longtitude(L)");
subplot(3,1,3);
hold on;
%plot(t,h);plot(rt,rh);legend("my","ref");
plot(t(1:size(rt,1)),h(1:size(rt,1))-rh);
hold off;
title("Height(H)");

figure(2)
subplot(3,1,1);
hold on;
%plot(t,vn);plot(rt,rvn);legend("my","ref");
plot(t(1:size(rt,1)),vn(1:size(rt,1))-rvn);
hold off;
title("V north");
subplot(3,1,2);
hold on;
%plot(t,ve);plot(rt,rve);legend("my","ref");
plot(t(1:size(rt,1)),ve(1:size(rt,1))-rve);
hold off;
title("V east");
subplot(3,1,3);
hold on;
%plot(t,vd);plot(rt,rvd);legend("my","ref");
plot(t(1:size(rt,1)),vd(1:size(rt,1))-rvd);
hold off;
title("V down");

figure(3)
subplot(3,1,1);
hold on;
%plot(t,roll);plot(rt,rroll);legend("my","ref");
plot(t(1:size(rt,1)),roll(1:size(rt,1))-rroll);
hold off;
title("roll");
subplot(3,1,2);
hold on;
%plot(t,pitch);plot(rt,rpitch);legend("my","ref");
plot(t(1:size(rt,1)),pitch(1:size(rt,1))-rpitch);
hold off;
title("pitch");
subplot(3,1,3);
hold on;
for i=1:size(rt,1)
    if abs(yaw(i)-ryaw(i))>100
        if yaw(i)>ryaw(i)
            yaw(i)=yaw(i)-360;
        else
            yaw(i)=yaw(i)+360;
        end
    end
end
%plot(t,yaw);plot(rt,ryaw);legend("my","ref");
plot(t(1:size(rt,1)),yaw(1:size(rt,1))-ryaw);
hold off;
title("yaw");
