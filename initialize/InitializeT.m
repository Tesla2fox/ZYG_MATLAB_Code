function [T]=InitializeT
h=6;
T(:,1)=round(rand(h,1)*20);
T(:,2)=round(rand(h,1)*20);
T(:,3)=(round(rand(h,1)*8)+1)/10;
T(:,4)=round(rand(h,1)*15)+5;
