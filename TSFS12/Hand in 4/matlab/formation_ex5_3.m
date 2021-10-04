idx = {1:2, 3:4, 5:6, 7:8, 9:10, 11:12, 13:14, 15:16};

agent(1).measpar.idx = idx{1};
agent(2).measpar.idx = cat(3,[idx{2};idx{1}],[idx{2};idx{3}],[idx{2};idx{4}]);
agent(3).measpar.idx = cat(3,[idx{3};idx{1}],[idx{3};idx{2}],[idx{3};idx{4}],[idx{3};idx{5}]);
agent(4).measpar.idx = cat(3,[idx{4};idx{2}],[idx{4};idx{3}],[idx{4};idx{5}],[idx{4};idx{6}]);
agent(5).measpar.idx = cat(3,[idx{5};idx{3}],[idx{5};idx{4}],[idx{5};idx{6}],[idx{5};idx{7}]);
agent(6).measpar.idx = cat(3,[idx{6};idx{4}],[idx{6};idx{5}],[idx{6};idx{7}],[idx{6};idx{8}]);
agent(7).measpar.idx = cat(3,[idx{7};idx{5}],[idx{7};idx{6}],[idx{7};idx{8}]);
agent(8).measpar.idx = cat(3,[idx{8};idx{6}],[idx{8};idx{7}]);
%agent(8).measpar.idx = idx{8};

agent(1).xref = @(t) [2*cos(t) 6+2*sin(t)]';
agent(2).xref = @(t) [1 2 0;1 0 -2];
agent(3).xref = @(t) [-1 -2 -2 0;1 0 -2 -2];
agent(4).xref = @(t) [0 2 2 0;2 2 0 -2];
agent(5).xref = @(t) [0 -2 -2 0; 2 0 -2 -2];
agent(6).xref = @(t) [0 2 2 1; 2 2 0 -1];
agent(7).xref = @(t) [0 -2 -1; 2 0 -1];
agent(8).xref = @(t) [-1 1; 1 1]';
%agent(8).xref = @(t) [2*cos(t) 2*sin(t)]';

