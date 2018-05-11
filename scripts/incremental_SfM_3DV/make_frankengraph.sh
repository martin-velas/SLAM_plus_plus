#!/bin/bash
cat graph_SE3_XYZ_global_2.txt > frankengraph.txt
echo SEPARATOR >> frankengraph.txt
cat graph_Sim3_INVD_relative_2.txt >> frankengraph.txt
cat frankengraph.txt | awk 'BEGIN { had_sep = 0; } { if(!had_sep) { if($1 == "SEPARATOR") had_sep = 1; else if($1 == "VERTEX_XYZ") { table[$2] = $0; } } else if($1 == "VERTEX:INVD") print table[$2]; else print $0; }' > frankengraph2.txt
mv frankengraph2.txt frankengraph.txt


#cat graph_Sim3_INVD_relative_2.txt > frankengraph.txt
#echo SEPARATOR >> frankengraph.txt
#cat graph_SE3_XYZ_global_2.txt >> frankengraph.txt
#cat frankengraph.txt | awk 'BEGIN { had_sep = 0; } { if(!had_sep) { if($1 == "SEPARATOR") had_sep = 1; else if($1 == "VERTEX_CAM:SIM3") { table[$2] = $0; } } else if($1 == "VERTEX_CAM") print table[$2]; else print $0; }' > frankengraph2.txt
#mv frankengraph2.txt frankengraph.txt
