clear all
res = 360;                              %Angle Resolution
x = 0:360/res:360;                       %individual angle values
L = length(x);                          %length of the array
r = zeros(1,L);                     %Results
rx = zeros(1,L);
RangeLimits = zeros(1,L);           %Range limits for use in point generation
%PLEASE MAKE SURE THE VERTICES LINK UP, OR ELSE THE GENERATION WONT WORK
verts = [0,-1;1,0;0,1;-1,0;0,-1];          %Vertices, in this case, its a rotated square
%verts = [0,-1;5,4;4,5;-1,0;0,-1];           %Rotated oblong ->
%verts = [0,-1;2,1;1,2;-1,0;0,-1];          %Rotated oblong UpRight
%verts = [0,-1;1,0;-1,2;-2,1;0,-1];         %Rotated UpLeft
%verts = [1,-2;2,-1;0,1;-1,0;1,-2];         %Oblong DownRight
CtrPt = 1/2*[sum([verts(2,1) verts(4,1)]) sum([verts(1,2) verts(3,2)])];
Slopes = zeros(4,2);                %Pre-allocate the slope array m,c
XAngles = zeros(1,4);
%Get slopes from the vertices (assume 1->2, 2->3, 3->4, 4->1)
VecCon = [1,2;2,3;3,4;4,1];
s='stop';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%GENERATE THE RANGE OUTLINE%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=1:4
    %Get the slope for each of the 4 lines, based on how they are connected
    Slopes(VecCon(i,1),1) = (verts(VecCon(i,2),1) - verts(VecCon(i,1),1))/(verts(VecCon(i,2),2) - verts(VecCon(i,1),2));
    
    %Get the intercepts for each of the 4 lines
    Slopes(i,2) = verts(i,2)-Slopes(i,1)*verts(i,1);
    
    XAngles(i) = atan2d(verts(i,2),verts(i,1));
    if XAngles(i) <0
        XAngles(i) = XAngles(i)+360;
    end
end
SortXAngles = sort(XAngles,'ascend');
LUI = 0;

for i=1:L
    %Find the slope and intercept for the given angle
    m = tand(x(i));
    c = 0;
    xcepts = zeros(1,2);
    
    
    if x(i) <= SortXAngles(1)
        LUI = 1;
    elseif x(i) <= SortXAngles(2) && x(i) > SortXAngles(1)
        LUI = 2;
    elseif x(i) <= SortXAngles(3) && x(i) > SortXAngles(2)
        LUI = 3;
    elseif x(i) <= SortXAngles(4) && x(i) > SortXAngles(3)
        LUI = 4;
    elseif x(i) > SortXAngles(4)
        LUI = 1;
    end
    
    switch x(i)
        case 90
            xcepts(1) = 0;
            xcepts(2) = Slopes(VecCon(:,2)==find(XAngles==SortXAngles(LUI)),1)*xcepts(1)+Slopes(VecCon(:,2)==find(XAngles==SortXAngles(LUI)),2);
        case 180
            xcepts(1) = (Slopes(VecCon(:,2)==find(XAngles==SortXAngles(LUI)),2)-c)/(m-Slopes(VecCon(:,2)==find(XAngles==SortXAngles(LUI)),1));
            xcepts(2) = 0;
        case 270
            xcepts(1) = 0;
            xcepts(2) = Slopes(VecCon(:,2)==find(XAngles==SortXAngles(LUI)),1)*xcepts(1)+Slopes(VecCon(:,2)==find(XAngles==SortXAngles(LUI)),2);
        case 360
            xcepts(1) = (Slopes(VecCon(:,2)==find(XAngles==SortXAngles(LUI)),2)-c)/(m-Slopes(VecCon(:,2)==find(XAngles==SortXAngles(LUI)),1));
            xcepts(2) = 0;
        case 0
            xcepts(1) = (Slopes(VecCon(:,2)==find(XAngles==SortXAngles(LUI)),2)-c)/(m-Slopes(VecCon(:,2)==find(XAngles==SortXAngles(LUI)),1));
            xcepts(2) = 0;
        otherwise
            xcepts(1) = (Slopes(VecCon(:,2)==find(XAngles==SortXAngles(LUI)),2)-c)/(m-Slopes(VecCon(:,2)==find(XAngles==SortXAngles(LUI)),1));
            xcepts(2) =  m*xcepts(1)+c;
    end
    
    RangeLimits(i) = sqrt((xcepts(1))^2+(xcepts(2))^2);
end

%Random point generation validation
invalidPt = true;
while invalidPt
    pos = [5*rand-1 5*rand-1];
    AfromO = atan2d(pos(2),pos(1)); %This is the angle the random point makes from the origin
    if AfromO <0
       AfromO = AfromO+360; 
    end
    if sqrt((pos(2))^2+(pos(1))^2) <RangeLimits((res/360)*round(AfromO)+1)  %Checking to see if the point is within the range limit
        invalidPt = false;              %(1-sind(45))*-abs(sind(2*(AfromO)))+1: old way of checking
    end
end

for i=1:4
    %Get the slope for each of the 4 lines, based on how they are connected
    Slopes(VecCon(i,1),1) = (verts(VecCon(i,2),1) - verts(VecCon(i,1),1))/(verts(VecCon(i,2),2) - verts(VecCon(i,1),2));
    
    %Get the intercepts for each of the 4 lines
    Slopes(i,2) = verts(i,2)-Slopes(i,1)*verts(i,1);
    
    XAngles(i) = atan2d(verts(i,2)-pos(2),verts(i,1)-pos(1)); %atan2d(verts(VecCon(i,2),2)-pos(2),verts(VecCon(i,1),1)-pos(1));
    if XAngles(i) <0
        XAngles(i) = XAngles(i)+360;
    end
end
SortXAngles = sort(XAngles,'ascend');
LUI = 0;

pMinX = min(verts(:,1));
pMaxX = max(verts(:,1));
pMinY = min(verts(:,2));
pMaxY = max(verts(:,2));


for i=1:L
    %Find the slope and intercept for the given angle
    m = tand(x(i));
    c = pos(2)-m*pos(1);
    xcepts = zeros(1,2);
    
    
    if x(i) <= SortXAngles(1)
        LUI = 1;
    elseif x(i) <= SortXAngles(2) && x(i) > SortXAngles(1)
        LUI = 2;
    elseif x(i) <= SortXAngles(3) && x(i) > SortXAngles(2)
        LUI = 3;
    elseif x(i) <= SortXAngles(4) && x(i) > SortXAngles(3)
        LUI = 4;
    elseif x(i) > SortXAngles(4)
        LUI = 1;
    end
    
    switch x(i)
        case 90
            xcepts(1) = pos(1);
            xcepts(2) = Slopes(VecCon(:,2)==find(XAngles==SortXAngles(LUI)),1)*xcepts(1)+Slopes(VecCon(:,2)==find(XAngles==SortXAngles(LUI)),2);
        case 180
            xcepts(1) = (Slopes(VecCon(:,2)==find(XAngles==SortXAngles(LUI)),2)-c)/(m-Slopes(VecCon(:,2)==find(XAngles==SortXAngles(LUI)),1));
            xcepts(2) = pos(2);
        case 270
            xcepts(1) = pos(1);
            xcepts(2) = Slopes(VecCon(:,2)==find(XAngles==SortXAngles(LUI)),1)*xcepts(1)+Slopes(VecCon(:,2)==find(XAngles==SortXAngles(LUI)),2);
        case 360
            xcepts(1) = (Slopes(VecCon(:,2)==find(XAngles==SortXAngles(LUI)),2)-c)/(m-Slopes(VecCon(:,2)==find(XAngles==SortXAngles(LUI)),1));
            xcepts(2) = pos(2);
        case 0
            xcepts(1) = (Slopes(VecCon(:,2)==find(XAngles==SortXAngles(LUI)),2)-c)/(m-Slopes(VecCon(:,2)==find(XAngles==SortXAngles(LUI)),1));
            xcepts(2) = pos(2);
        otherwise
            xcepts(1) = (Slopes(VecCon(:,2)==find(XAngles==SortXAngles(LUI)),2)-c)/(m-Slopes(VecCon(:,2)==find(XAngles==SortXAngles(LUI)),1));
            xcepts(2) =  m*xcepts(1)+c;
    end
    
    r(i) = sqrt((pos(1)-xcepts(1))^2+(pos(2)-xcepts(2))^2);
    rx(i) = awgn(r(i), 400);    % set this up as the received signal
    
    animateplot = false;
    if animateplot
        vis = zeros(101,2);
        if size(pos(1):(xcepts(1)-pos(1))/100:(xcepts(1))) ~= 0
            vis(:,1) = pos(1):(xcepts(1)-pos(1))/100:(xcepts(1));
            vis(:,2) = vis(:,1)*m+c;
            %subplot(3,1,3);
            plot(verts(:,1),verts(:,2),pos(1),pos(2),'ro',vis(:,1),vis(:,2),xcepts(1),xcepts(2),'x',rx(1:i).*cosd(x(1:i))+pos(1), rx(1:i).*sind(x(1:i))+pos(2),'kx');
        end
        title('Animated Scan');
        
        hold on;
        hold off;
        axis([pMinX, pMaxX, pMinY, pMaxY]);
        
%         subplot(3,1,2)
%         plot(diff(r(1:i)));
%         title('rate of change of slope');
%         axis([0,length(x),min(diff(r)), max(diff(r))]);
%         
%         subplot(3,1,1);
%         plot(x,r);
%         title('2D Point cloud');
%         ylabel('Distance');
%         xlabel('angle');
%         axis([0,length(x),min(r),max(r)]);
        pause(0.05);
    end
end


%THIS SUCCESSFULLY CREATES THE 2D POINT CLOUD AT A GIVEN POSITION
%GOD BLESS

%FROM THIS POINT ON, THE ONLY VARIABLES WE CAN ACCESS IS RX, & X



%NOW WE NEED TO DO THE HARD STUFF
%From this point on, we want to rely purely on the results to generate
%position, as this is how it would work practically

%First thing i need to do is locate the indexes of the 4 sheer points
%We need to know how the results change over time, so we differentiate the
%results matrix
% drda = diff(rx);
% %Sheers = zeros(1,4);
% SheerCount = 1;
% for i=2:L-1
%     %If it goes from negative from positive, we can be pretty sure that
%     %we've found an area of sheer drop. SO we want to jot down its index
%     if drda(i) <=0 && drda(i-1)>=0 && i-1~=1
%         Sheers(SheerCount) = i;
%         SheerCount = SheerCount+1;
%     end
% end
%     %If the above loop doesnt get anymore than the amount of corners, then
%     %we can assume that it was filtered out at index 0 or something, so we
%     %want to add this back in
% if SheerCount == 4
%    Sheers(4) = length(rx); 
% end
%     
% %Use the unique function to get rid of double entries
% Sheers = unique(Sheers);
% 
% 
% % as this is an approximate index, we want to narrow it down to the
% % actual index. To do this, im going to look withing a few indexes
% % around the approximate one to find the local maxima, then have that
% % as the actual index
% for i=1:length(Sheers)
%     SW = 2;
%     %Boundary conditions
%     if Sheers(i)-SW < 0
%         LMaxima = max(rx(Sheers(i):Sheers(i)+SW));
%         Sheers(i) = find(rx(Sheers(i):Sheers(i)+SW)==LMaxima);
%     elseif Sheers(i)+SW >length(r)
%         LMaxima = max(rx(Sheers(i)-SW:Sheers(i)));
%         Sheers(i) = find(rx(Sheers(i)-SW:Sheers(i))==LMaxima)+Sheers(i)-(SW+1);
%     else
%         LMaxima = max(rx(Sheers(i)-SW:Sheers(i)+SW));    % local maxima at the approximate index
%         Sheers(i) = find(rx(Sheers(i)-SW:Sheers(i)+SW)==LMaxima)+Sheers(i)-(SW+1);
%     end
% end
% 
% %Now we have the indexes for each of the corners, which means we can assign
% %each of the corners a polar co-ordinate. This means that we need to get
% %the angle at each of the indexes
% PtApprox = zeros(4,2);  %Approximation of the points based on their polar coords.
% for i=1:length(Sheers)
%     PtApprox(i,1) = rx(Sheers(i))*cosd(x(Sheers(i)));
%     PtApprox(i,2) = rx(Sheers(i))*sind(x(Sheers(i)));
% end
% % 
% % for i =1:4
% %    PtApprox(i,1) = r(i*(res/4)+1)*cosd(x(i*(res/4)+1));
% %    PtApprox(i,2) = r(i*(res/4)+1)*sind(x(i*(res/4)+1));
% % end
% 
% %We might just be able to approximate our position now
% PosApprox = [-sum(PtApprox(:,1))/(SheerCount-1) -sum(PtApprox(:,2))/(SheerCount-1)];
% %PosApprox = [-sum(PtApprox(:,1))/4 -sum(PtApprox(:,2))/4];
% 
% endplot = true;
% if endplot
%     subplot(3,1,1);
%     plot(rx);
%     axis([0,length(x),0,max(r)]);
%     for i=1:length(Sheers)
%         xline(Sheers(i));
%     end
%     title('2D Point cloud');
%     ylabel('Distance');
%     xlabel('angle');
%     
%     subplot(3,1,2);
%     plot(drda);
%     title('rate of change of slope');
%     axis([0,length(x),min(drda),max(drda)]);
%     
%     subplot(3,1,3);
%     plot(verts(:,1),verts(:,2),pos(1),pos(2),'ok', PosApprox(1), PosApprox(2),'xr');
%     title('Approximate vs Actual position');
% end

%ALL OF THE ABOVE CODE IS MAKING THIS WAY MORE COMPLICATED THAN IT NEEDS TO
%BE
[~, HMax] = max(rx.*cosd(x));
[~, HMin] = min(rx.*cosd(x));
[~, VMax] = max(rx.*sind(x));
[~, VMin] = min(rx.*sind(x));


PtApprox = [rx(HMax).*cosd(x(HMax)) rx(HMax).*sind(x(HMax)); 
            rx(HMin).*cosd(x(HMin)) rx(HMin).*sind(x(HMin)); 
            rx(VMax).*cosd(x(VMax)) rx(VMax).*sind(x(VMax)); 
            rx(VMin).*cosd(x(VMin)) rx(VMin).*sind(x(VMin))];
Vert1 =(sum([sqrt((PtApprox(4,1)-PtApprox(1,1))^2 + (PtApprox(4,2)-PtApprox(1,2))^2) sqrt((PtApprox(2,1)-PtApprox(3,1))^2 + (PtApprox(2,2)-PtApprox(3,2))^2)])/2);
Vert2 = (sum([sqrt((PtApprox(1,1)-PtApprox(3,1))^2 + (PtApprox(1,2)-PtApprox(3,2))^2) sqrt((PtApprox(2,1)-PtApprox(4,1))^2 + (PtApprox(2,2)-PtApprox(4,2))^2)])/2);
%Room Ratio Offset, average of 2 opposite sides, divided by the other
%HPol = atan2d(sum(PtApprox(1:2,1)), sum(PtApprox(1:2,2)));
%VPol = atan2d(sum(PtApprox(3:4,1)), sum(PtApprox(3:4,2)));


HPol = HMax-HMin;
VPol = VMax-VMin;
HPol = HPol/abs(HPol);
VPol = VPol/abs(VPol);
Pol = [HPol VPol];

%RRO = min([Vert1 Vert2])/max([Vert1 Vert2]);
RRO1 = Vert1/Vert2;
RRO2 = Vert2/Vert1;

% I THINK EVERYTHING I'VE JUST DONE IS GARBAGE
% I ALSO THINK I NEED TO FIND THE CENTRE POINT OF THE ROOM, AND THEN OFFSET
% MY RESULT BY THAT

CentrePoint = 1/2*[sum([PtApprox(1,1) PtApprox(2,1)]) sum([PtApprox(3,2) PtApprox(4,2)])];


XLines = [sqrt((PtApprox(1,1)-PtApprox(2,1))^2+(PtApprox(1,2)-PtApprox(2,2))^2) sqrt((PtApprox(3,1)-PtApprox(4,1))^2+(PtApprox(3,2)-PtApprox(4,2))^2)]; %The length of the line that intersects the centrepoint from the horizontal and vertical extrema
%TD = [rx(HMax)+rx(HMin) rx(VMax)+rx(VMin)]; %Total distance from between the approximated vertices in their respective polarity
PosApprox = [PtApprox(2,1)+rx(HMin)*cosd(180+x(HMin)) PtApprox(4,2)+rx(VMin)*sind(180+x(VMin))];
plotadjusted=true;
if plotadjusted
    plot(verts(:,1),verts(:,2),rx.*cosd(x)+pos(1), rx.*sind(x)+pos(2),'kx',pos(1),pos(2),'rd',PosApprox(1)+pos(1), PosApprox(2)+pos(2),'go', CentrePoint(1)+pos(1), CentrePoint(2)+pos(2), 'gh', CtrPt(1), CtrPt(2), 'rh');
else
    plot(verts(:,1),verts(:,2),rx.*cosd(x), rx.*sind(x),'kx',pos(1),pos(2),'ro',PosApprox(1), PosApprox(2),'go', CentrePoint(1), CentrePoint(2), 'gh', CtrPt(1), CtrPt(2), 'rh');
end
grid on;
title('Full Scan with Approximate Position Vs Actual Position');


