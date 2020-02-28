clear all
port = 42072;
t = tcpclient('129.127.225.70', port);  %Host of the TCP server
%u = udp('129.127.225.142',2390);
%fopen(u);
i=1;
while true
    data = char(read(t,45));
    
    s = size(strfind(data,'/'),2);
    for j=1:s
        idx = strfind(data,'/');
        str = data(1:idx(1)-1);
        RxData(i,j) = str2double(str);
        data = eraseBetween(data,1,idx(1));
    end
    str = data;
    RxData(i,5) = str2double(str);
    
    i=i+1;
    %fwrite(u,"0,0,20,10,360")
    plot(RxData(:,4).*cosd(RxData(:,3)),RxData(:,4).*sind(RxData(:,3)),'x')
end