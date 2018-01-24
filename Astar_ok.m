%% A*路径规划算法
%%by xiongzhan 20170325
clc;
clear;
% pause(3); 
%% 初始数据
n=30;
starNum=1;
% starNum = randi(n*n,[1,1]);
goalNum=172;
% goalNum = randi(n*n,[1,1]);
banper=0.25;
%% 地图初始化
figure('name','A*','NumberTitle','off','MenuBar','none');
global point 
% 将point作为一个公共数据库，使子函数也能使用数据。
for ii=1:n*n
    point(ii).num = ii;
    point(ii).father=[];
    point(ii).Gcost=[];
    point(ii).Hcost=[];
end
%% 设置障碍
banList=[randi(n*n,[1,floor(banper*n*n)])];
load banList
banList(find(banList==goalNum))=[];
for jj = 1:length(banList)
    if banList(jj)~=goalNum || banList(jj)~=starNum
        point(banList(jj)).Gcost = Inf;
    end
end
point(starNum).Gcost=0;
point(starNum).father = point(starNum).num;
point(starNum).Hcost=getHcost(point(starNum),point(goalNum),n);

%% A*算法核心
openList = [];
closeList = [];
closeListNum=[];
openListNum=[];
openList = [openList,point(starNum)];
while length(openList)
    % opneList是待被检测的节点，为空则说明无解
    costList = getCost(openList,point(goalNum),n);
    % 计算openList中节点的代价，以代价最小的节点为当前节点，继续向外扩展
    currentPoint = openList(find(costList==min(costList),1));
    openList(find(min(costList)==costList,1))=[];
    closeList = [closeList,currentPoint];
    neighbourNum = getNeighbour(currentPoint,n);
    closeListNum = cat(1,closeList.num);
    openListNum = cat(1,openList.num);
    for ii = 1:length(neighbourNum)
%         ne1 = neighbourNum(ii);
%         if ne1==146 && currentPoint.num==166
%             ne1
%         end
        if neighbourNum(ii)==point(goalNum).num
            point(neighbourNum(ii)).father = currentPoint.num;
            point(goalNum).father = currentPoint.num;
            disp('ok')
            routPlot(goalNum,n);
            return;
        end
            %首先判断是否是障碍物/closeList
            log1=0;
            try
                tmp=point(neighbourNum(ii)).Gcost;
                if tmp ==inf
                    log1 = 1;
                end
            catch
                log1=0;
            end
            %判断是否在closeListNum中，如果在跳过。
            if log1 || ismember(neighbourNum(ii),closeListNum)
                continue;
            elseif (ismember(neighbourNum(ii),openListNum))
            %判断是否在openList中，如果在要分情况考虑，判断节点新路径的G代价是否小于原路径
            %neighbourNum(ii)在openList中，若以neighbourNum(ii)为父节点Gcost小，则用neighbourNum(ii)为父节点。
            oldGcost = getGcost(point(neighbourNum(ii)),n);
            father = point(neighbourNum(ii)).father;
            point(neighbourNum(ii)).father = currentPoint.num;
            newGcost = getGcost(point(neighbourNum(ii)),n);
            if newGcost>oldGcost
                %代价大于原路径，将父节点重置。
                point(neighbourNum(ii)).father = father;
            else
                %代价小于原路径，将父节点用当前节点替代，修改Gcost。
                point(neighbourNum(ii)).Gcost = newGcost;
            end
            continue;
        elseif ~ismember(neighbourNum(ii),closeListNum)
            %前面的情况都排除时，将节点加入到openList并储存父节点、代价等信息。
            point(neighbourNum(ii)).father = currentPoint.num;
            point(neighbourNum(ii)).Gcost = getGcost(point(neighbourNum(ii)),n);
            point(neighbourNum(ii)).Hcost = getHcost(point(neighbourNum(ii)),point(goalNum),n);
            openList = [openList,point(neighbourNum(ii))];
            end
    end
    closeListNum = cat(1,closeList.num);
    openListNum = cat(1,openList.num);
    pause(0.1);
    mydrawnow(starNum,goalNum,banList,closeListNum,openListNum,n);
end

