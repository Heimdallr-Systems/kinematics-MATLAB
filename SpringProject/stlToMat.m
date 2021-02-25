clear

[Body, Body_f, ~, ~, ~] = stlread('Body.stl');
[FLLink1, FLLink1_f, ~, ~, ~] = stlread('Link_1_FL.stl');
[FLLink2, FLLink2_f, ~, ~, ~] = stlread('Link_2_FL.stl');
[FLLink3, FLLink3_f, ~, ~, ~] = stlread('Link_3_FL.stl');
[FRLink1, FRLink1_f, ~, ~, ~] = stlread('Link_1_FR.stl');
[FRLink2, FRLink2_f, ~, ~, ~] = stlread('Link_2_FR.stl');
[FRLink3, FRLink3_f, ~, ~, ~] = stlread('Link_3_FR.stl');
[BRLink1, BRLink1_f, ~, ~, ~] = stlread('Link_1_BR.stl');
[BRLink2, BRLink2_f, ~, ~, ~] = stlread('Link_2_BR.stl');
[BRLink3, BRLink3_f, ~, ~, ~] = stlread('Link_3_BR.stl');
[BLLink1, BLLink1_f, ~, ~, ~] = stlread('Link_1_BL.stl');
[BLLink2, BLLink2_f, ~, ~, ~] = stlread('Link_2_BL.stl');
[BLLink3, BLLink3_f, ~, ~, ~] = stlread('Link_3_BL.stl');

save model.mat

clear

load model.mat