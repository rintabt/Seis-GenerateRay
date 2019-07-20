function [G,d,KoordRay]=DefineAllRaypath(Model_V,SR,dx)
% Function ini untuk menelusuri raypath untuk semua rec dan semua src.
% Looping penelusuran raypath per-rec per-src dilakukan dengan function
% TracingBack.m
%
% structure SR : src_x, src_z, rec_x, rec_z
%
% KoordRay{Temp.src_ke,Temp.rec_ke}
% KoordRay dalam bentuk cell untuk setiap src-rec. Isi dalam cell tersebut
% adalah koordinat subscript ([x z]) titik-titik dari raypath. 


N.src = length(SR.src_x);
N.rec = length(SR.rec_x);
N.j = numel(Model_V);
Model.batas_ind_b = 200;
Model.V = Model_V;
Model.h = dx;
Model.sz = size(Model.V);

Temp.src_ke = 0;
Indeks.i = 0;
while Temp.src_ke < N.src % Forward (per-src) & Telusur raypath (per-rec)
    Temp.src_ke = Temp.src_ke + 1;
    Temp.XS = round(SR.src_x(Temp.src_ke));
    Temp.ZS = round(SR.src_z(Temp.src_ke));
    
    Temp.time = eikonal2D(Model.V, Model.h, Temp.XS, Temp.ZS);
    [Temp.teta,Temp.ket] = Cal_Teta(Temp.time,Model);
    
%     Ray.time_2D {Temp.src_ke,1} = Temp.time;
%     Ray.teta {Temp.src_ke,1} = Temp.teta;
        
    Temp.rec_ke = 0;
    while Temp.rec_ke < N.rec %Telusur raypath (per-rec)
        Indeks.i = Indeks.i + 1;
        
        Temp.rec_ke = Temp.rec_ke + 1;
        Temp.XR = SR.rec_x(Temp.rec_ke);
        Temp.ZR = SR.rec_z(Temp.rec_ke);

        [Temp] = TracingBack (Model,N,Temp);
        Eq.d(Indeks.i) = Temp.d;
        Eq.G(Indeks.i,:) = Temp.G;
        
        Ray.KoordRay{Temp.src_ke,Temp.rec_ke} = Temp.KoordRay;
    end
end

G = Eq.G;
d = Eq.d;
KoordRay = Ray.KoordRay;
