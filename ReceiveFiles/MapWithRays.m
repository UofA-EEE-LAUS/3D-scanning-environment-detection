clear all
res = 720;                              %Angle Resolution
CohnAngle = 27;                         %Angle Width of the scanner
xb = 0:360/res:360;                     %individual angle values
xp = xb+CohnAngle/2;                    %angle with the added cone range
xn = xb-CohnAngle/2;
x = [xn; xb; xp];
L = length(x);                          %length of the array
r = zeros(1,L);                         %Results
rx = zeros(1,L);
RangeLimits = zeros(1,L);               %Range limits for use in point generation
%PLEASE MAKE SURE THE VERTICES LINK UP, OR ELSE THE GENERATION WONT WORK
verts = [0,-1;1,0;0,1;-1,0;0,-1];       %Vertices, in this case, its a rotated square
%verts = [0,-1;5,4;4,5;-1,0;0,-1];      %Rotated oblong ->
%verts = [0,-1;2,1;1,2;-1,0;0,-1];      %Rotated oblong UpRight
%verts = [0,-1;1,0;-1,2;-2,1;0,-1];     %Rotated UpLeft
%verts = [1,-2;2,-1;0,1;-1,0;1,-2];     %Oblong DownRight
CtrPt = 1/2*[sum([verts(2,1) verts(4,1)]) sum([verts(1,2) verts(3,2)])];
Slopes = zeros(4,2);                    %Pre-allocate the slope array m,c
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
        XAngles(i) = XAngles(i)+360;    %Convert to 0 -> 360
    end
end
SortXAngles = sort(XAngles,'ascend');
LUI = 0;    %The line that we are inspecting

for i=1:L
    %Find the slope and intercept for the given angle
    m = tand(x(2,i));
    c = 0;                  %As we are looking from the origin, this is where we set the intercept
    xcepts = zeros(1,2);
    
    %Depending on the angle we're currently at, we'll be looking at a
    %different line, this if/else stuff decides which line we are looking
    %at
    if x(2,i) <= SortXAngles(1)
        LUI = 1;
    elseif x(2,i) <= SortXAngles(2) && x(2,i) > SortXAngles(1)
        LUI = 2;
    elseif x(2,i) <= SortXAngles(3) && x(2,i) > SortXAngles(2)
        LUI = 3;
    elseif x(2,i) <= SortXAngles(4) && x(2,i) > SortXAngles(3)
        LUI = 4;
    elseif x(2,i) > SortXAngles(4)
        LUI = 1;
    end
    
    %I was having some issues at the n*90 degree angles, so I just
    %hardcoded what they should be. In a perfect world, id be able to
    %figure out why that was happening. But this isnt a perfect world
    switch x(2,i)
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
            %Here is the crux of how it finds the x-intercept. Its
            %basically (c1 -c2)/(m2-m1), but with a lot more name
            %inclusions to make sure that its picking the right ones. then
            %the y-intercept is simply y=mx+c
            xcepts(1) = (Slopes(VecCon(:,2)==find(XAngles==SortXAngles(LUI)),2)-c)/(m-Slopes(VecCon(:,2)==find(XAngles==SortXAngles(LUI)),1));
            xcepts(2) =  m*xcepts(1)+c;
    end
    
    %This gets the maximum range that our random point can be away from the
    %origin to ensure that locations will be generated inside of the square
    RangeLimits(i) = sqrt((xcepts(1))^2+(xcepts(2))^2);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%Random point generation validation%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

invalidPt = true;
while invalidPt
    pos = 5*[rand-0.5 rand-0.5];    %Randomly generates an xy co-ordinate
    AfromO = atan2d(pos(2),pos(1)); %This is the angle the random point makes from the origin
    if AfromO <0
        AfromO = AfromO+360;
    end
    if sqrt((pos(2))^2+(pos(1))^2) <RangeLimits((res/360)*round(AfromO)+1)  %Checking to see if the point is within the range limit at the respective index
        invalidPt = false;
    end
end

%Manual position allocation
%This can be commented out
%pos = [0.5 0];

for i=1:4
    %Get the slope for each of the 4 lines, based on how they are connected
    Slopes(VecCon(i,1),1) = (verts(VecCon(i,2),1) - verts(VecCon(i,1),1))/(verts(VecCon(i,2),2) - verts(VecCon(i,1),2));
    
    %Get the intercepts for each of the 4 boundary lines
    Slopes(i,2) = verts(i,2)-Slopes(i,1)*verts(i,1);
    
    XAngles(i) = atan2d(verts(i,2)-pos(2),verts(i,1)-pos(1));
    if XAngles(i) <0
        XAngles(i) = XAngles(i)+360;    %Convert to 0 -> 360
    end
end
SortXAngles = sort(XAngles,'ascend');
LUI = 0;

%The point minima and maxima, used for plotting purposes
pMinX = min(verts(:,1));
pMaxX = max(verts(:,1));
pMinY = min(verts(:,2));
pMaxY = max(verts(:,2));
SNR = 400; %How much noise we want to introduce into the received signal

for i=1:L
    for j=1:3
        %Find the slope and intercept for the given angle for our position
        m = tand(x(j,i));
        c = pos(2)-m*pos(1);
        xcepts = zeros(1,2);
        
        %Depending on the angle we're currently at, we'll be looking at a
        %different line, this if/else stuff decides which line we are looking
        %at
        if x(j,i) <= SortXAngles(1)
            LUI = 1;
        elseif x(j,i) <= SortXAngles(2) && x(j,i) > SortXAngles(1)
            LUI = 2;
        elseif x(j,i) <= SortXAngles(3) && x(j,i) > SortXAngles(2)
            LUI = 3;
        elseif x(j,i) <= SortXAngles(4) && x(j,i) > SortXAngles(3)
            LUI = 4;
        elseif x(j,i) > SortXAngles(4)
            LUI = 1;
        end
        
        %I was having some issues at the n*90 degree angles, so I just
        %hardcoded what they should be. In a perfect world, id be able to
        %figure out why that was happening. But this isnt a perfect world
        switch x(j,i)
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
                %Here is the crux of how it finds the x-intercept. Its
                %basically (c1 -c2)/(m2-m1), but with a lot more name
                %inclusions to make sure that its picking the right ones. then
                %the y-intercept is simply y=mx+c
                xcepts(1) = (Slopes(VecCon(:,2)==find(XAngles==SortXAngles(LUI)),2)-c)/(m-Slopes(VecCon(:,2)==find(XAngles==SortXAngles(LUI)),1));
                xcepts(2) =  m*xcepts(1)+c;
        end
        
        %We can now get the range at that angle
        r(j,i) = sqrt((pos(1)-xcepts(1))^2+(pos(2)-xcepts(2))^2);
    end
    rx(i) = awgn(min(r(:,i)), SNR);    % set this up as the received signal
    
    %I have some animated plot code that looks nice if its enabled
    animateplot = false;
    if animateplot
        vis = zeros(101,2);
        if size(pos(1):(xcepts(1)-pos(1))/100:(xcepts(1))) ~= 0
            vis(:,1) = pos(1):(xcepts(1)-pos(1))/100:(xcepts(1));
            vis(:,2) = vis(:,1)*m+c;
            %subplot(3,1,3);
            plot(verts(:,1),verts(:,2),pos(1),pos(2),'ro',vis(:,1),vis(:,2),xcepts(1),xcepts(2),'x',rx(1:i).*cosd(xb(1:i))+pos(1), rx(1:i).*sind(xb(1:i))+pos(2),'kx');
        end
        title('Animated Scan');
        hold on;
        hold off;
        axis([pMinX, pMaxX, pMinY, pMaxY]);
        pause(0.01);
    end
end

%FROM THIS POINT ON, THE ONLY VARIABLES WE CAN ACCESS IS RX, & XB
%NOW WE NEED TO DO THE HARD STUFF
%From this point on, we want to rely purely on the results to generate
%position, as this is how it would work practically

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%DATA PROCESSING AREA%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Indexes of where the Horizontal and Vertical Extrema appear
[~, HMax] = max(rx.*cosd(xb));
[~, HMin] = min(rx.*cosd(xb));
[~, VMax] = max(rx.*sind(xb));
[~, VMin] = min(rx.*sind(xb));

PtApprox = [rx(HMax).*cosd(xb(HMax)) rx(HMax).*sind(xb(HMax));
    rx(VMax).*cosd(xb(VMax)) rx(VMax).*sind(xb(VMax));
    rx(HMin).*cosd(xb(HMin)) rx(HMin).*sind(xb(HMin));
    rx(VMin).*cosd(xb(VMin)) rx(VMin).*sind(xb(VMin));
    rx(HMax).*cosd(xb(HMax)) rx(HMax).*sind(xb(HMax))];

%Lengths of the 4 estimated sides, in the order of HMax->VMax->HMin->VMin
EstVerts = zeros(1,4);
for i=1:4
    EstVerts(i) = sqrt((PtApprox(VecCon(i,1),1) - PtApprox(VecCon(i,2),1))^2 + (PtApprox(VecCon(i,1),2) - PtApprox(VecCon(i,2),2))^2);
end
P = sum(EstVerts);
[MinDist, MinIndex] = min(rx);
EstVertsT = EstVerts';
DiffMTXP = EstVerts./EstVertsT;
DiffMTX = abs(1-(DiffMTXP));
MaxRow = [sum(DiffMTX(1,:)) 1];
for i = 1:size(DiffMTX,1)
    if sum(DiffMTX(i,:))>MaxRow(1)
        MaxRow(1) = sum(DiffMTX(i,:));
        MaxRow(2) = i;
    end
    for j = 1:size(DiffMTX,2)
        if DiffMTX(i,j) == 0 && i==j
            DiffMTX(i,j) = 100;
        end
    end
end

MaxCrossWidth = [max([DiffMTXP(1,3) DiffMTXP(3,1)]) max([DiffMTXP(2,4) DiffMTXP(4,2)])]; %maxima of opposite line length ratio
MinCrossWidth = [min([DiffMTXP(1,3) DiffMTXP(3,1)]) min([DiffMTXP(2,4) DiffMTXP(4,2)])]; %Minima of opposite line length ratio

%PropAdd = (-1*DiffMTXP(4,2)*EstVerts(4) - 1*DiffMTXP(2,4)*EstVerts(2))/P;
%PropAdd = 0;


PosApprox = [PtApprox(3,1)+rx(HMin)*cosd(180+xb(HMin)) PtApprox(4,2)+rx(VMin)*sind(180+xb(VMin))];


EstCtr = -1/L*[sum(rx.*cosd(xb)) sum(rx.*sind(xb))];


PropAdd = [0 0];
%PropAdd = zeros(4,2);
for i=1:4
   Angle = i*90+135;
   Mag = sqrt(DiffMTXP(mod(i+1,4)+1, i)/EstVerts(i));
   PropAdd = PropAdd + Mag*[cosd(Angle) sind(Angle)];
   %PropAdd(i,:) = Mag*[cosd(Angle) sind(Angle)];
end

CentrePoint = [sum([PtApprox(1,1) PtApprox(3,1)]) sum([PtApprox(2,2) PtApprox(4,2)])]+PropAdd;%+PropAdd;

plotadjusted=true;
if plotadjusted                                                                      
    OV = [pos(1) pos(2)];
    %plot(verts(:,1),verts(:,2),rx.*cosd(xb)+pos(1), rx.*sind(xb)+pos(2),'kx',pos(1),pos(2),'rd',PosApprox(1)+pos(1), PosApprox(2)+pos(2),'go', CentrePoint(1)+pos(1), CentrePoint(2)+pos(2), 'gh', CtrPt(1), CtrPt(2), 'rh', EstCtr(1), EstCtr(2), 'b+', PtApprox(:,1)+pos(1), PtApprox(:,2)+pos(2));
else
    OV = [0 0];
    %plot(verts(:,1),verts(:,2),rx.*cosd(xb), rx.*sind(xb),'kx',pos(1),pos(2),'ro',PosApprox(1), PosApprox(2),'go', CentrePoint(1), CentrePoint(2), 'gh', CtrPt(1), CtrPt(2), 'rh');
end
    plot(verts(:,1),verts(:,2),rx.*cosd(xb)+OV(1), rx.*sind(xb)+OV(2),'kx',pos(1),pos(2),'rd',PosApprox(1)+OV(1), PosApprox(2)+OV(2),'go', CentrePoint(1)+OV(1), CentrePoint(2)+OV(2), 'g^', CtrPt(1), CtrPt(2), 'rv', EstCtr(1), EstCtr(2), 'b+', PtApprox(:,1)+OV(1), PtApprox(:,2)+OV(2));

grid on;
title('Full Scan with Approximate Position Vs Actual Position');


