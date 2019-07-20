function [Temp] = TracingBack (model,N,Temp)
% Function ini untuk merunut atau menelusuri raypath dari 1 receiver ke
% 1 source. Penelusuran ini dilakukan dari posisi rec hingga ke src.
%
% Dibuat berdasarkan referensi 
% "Two-Dimensional Tomographic Inversion With Finite-Difference Traveltimes"
% Oldenburg (1993)

batas_ind_b = model.batas_ind_b;

if Temp.XR == Temp.XS && Temp.ZR == Temp.ZS
    Temp.d = 0;
    Temp.G = zeros(1,N.j);
    Temp.KoordRay = [Temp.XR Temp.ZR];
else 
    xa = Temp.XR;
    za = Temp.ZR;

    %1. Penentuan sel dan kondisi awal di (XR,ZR)
    if (mod(xa,1)==0) && (mod(za,1)==0) %Tepat di grid point
        [x,z,T,kondisi] = sel_titik_a (xa,za,model.sz,Temp.time);
    elseif (mod(xa,1)~=0) && (mod(za,1)~=0) %Di tengah sel
        [x,z,T,kondisi] = sel_tengah(xa,za,model.sz,Temp.time);
    else %Di sisi sel kiri/kanan/atas/bawah
        [x,z,T,kondisi] = sel_sisi(xa,za,model.sz,Temp.time);
    end
    Temp.d = T; %d_obs

    %2. Perunutan ray
    ind_b = 1;
    X = xa; % X dan Z nantinya akan menjadi vektor yang berisi
    Z = za; % titik-titik raypath pada setiap sel.
    xb = xa; %xb dan zb adalah titik akhir raypath pada setiap sel. 
    zb = za; %Nilainya berubah2 setiap iterasi sel.
    Temp.G = zeros(1,N.j);
    while (xb~=Temp.XS || zb~=Temp.ZS) && ind_b<batas_ind_b %Iterasi pencarian ray dilakukan hingga mencapai koordinat source
        ind_b = ind_b+1;
        [xb,zb,~,x2,z2,kondisi2,~] = raypath(x,z,xa,za,Temp,kondisi,model.sz,ind_b,batas_ind_b);
        X(ind_b,1) = xb;
        Z(ind_b,1) = zb;
        
        if xa~=xb || za~=zb %rayline (Membuat matriks G)
            [pilih_sel,~,r] = rayline(xa,za,xb,zb);
            r = r*model.h;
            for i = 1:length(r)
                ind_G = sub2ind(model.sz,pilih_sel(i,2),pilih_sel(i,1));
                Temp.G(ind_G) = Temp.G(ind_G) + r(i);
            end
        end

        x = x2; z = z2; xa = xb; za = zb;
        kondisi = kondisi2;
        
        if ind_b==200 %&& Pilih.display_log==1
            disp ([Temp.src_ke Temp.rec_ke])
        end
    end
    Temp.KoordRay = [X Z];
end