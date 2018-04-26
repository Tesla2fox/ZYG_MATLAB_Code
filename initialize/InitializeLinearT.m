function [T]=InitializeLinearT
h=4;
T(:,1)=round(rand(h,1)*20);
T(:,2)=round(rand(h,1)*20);
T(:,3)=zeros(h,1);
T(:,4)=round(rand(h,1)*15)+5;
