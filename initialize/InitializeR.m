function [R]=InitializeR
h=8;
R(:,1)=round(rand(h,1)*20);
R(:,2)=round(rand(h,1)*20);
R(:,3)=ones(h,1);
R(:,4)=(round(rand(h,1)*8)+1)/10;

