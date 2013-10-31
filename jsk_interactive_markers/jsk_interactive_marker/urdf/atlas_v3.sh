# add head link
cp `rospack find atlas_description`/urdf/atlas_v3.urdf .
patch -p0 < atlas_v3.urdf.patch