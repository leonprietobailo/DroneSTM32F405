T = readtable('data3.csv');
Z = table2array(T);
% figure;


for(i=size(Z, 1):-1:1)
    if(Z(i, 3) < 1250 || Z(i, 3) > 2000)
        Z(i,:) = [];
    end
end

figure, scatter(Z(:,2),Z(:,3));
hold on

hT_base = 1500;
for(i=1:size(Z, 1)-1)
    hT_base(i+1) = hT_base(i) * 0.999 + Z(i,3) * 0.001;
end

 scatter(Z(:,2),hT_base,'red');