/*

    Copyright Eli Dupree and Isaac Dupree, 2011, 2012

    This file is part of Lasercake.

    Lasercake is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Lasercake is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Lasercake.  If not, see <http://www.gnu.org/licenses/>.

*/


const int SUN_AREA_SIZE = 256;


void world::update_light(vector3<fine_scalar> sun_direction)
{
  litnesses_.clear();
  bool packets[SUN_AREA_SIZE][SUN_AREA_SIZE];
  for (int i = 0; i < SUN_AREA_SIZE; ++i) { for (int j = 0; j < SUN_AREA_SIZE; ++j) {
    packets[i][j] = true;
  }}
  
  for (each polyhedron p of each tile or object, in sun order) {
    int max_x, max_y, min_x, min_y;
    bool any = false;
    for (vector3<fine_scalar> const& v : p.get_vertices()) {
      vector3<fine_scalar> projected_vertex = v;
      projected_vertex[X] -= projected_vertex(Z) * sun_direction(X) / sun_direction(Z);
      projected_vertex[Y] -= projected_vertex(Z) * sun_direction(Y) / sun_direction(Z);
      projected_vertex[Z] = 0;
      
      projected_vertex[X] = projected_vertex(X) * 10 / tile_width;
      projected_vertex[Y] = projected_vertex(Y) * 10 / tile_width;
      if (!any || projected_vertex(X) > max_x) max_x = projected_vertex(X);
      if (!any || projected_vertex(Y) > max_y) max_y = projected_vertex(Y);
      if (!any || projected_vertex(X) < min_x) min_x = projected_vertex(X);
      if (!any || projected_vertex(Y) < min_y) max_y = projected_vertex(Y);
      any = true;
    }

    for (int x = min_x; x <= max_x; ++x) {
      for (int y = min_y; i <= max_y; ++i) {
        if (packets[x][y]) {
          ++litnesses_[the object or tile];
          packets[x][y] = false;
        }
      }
    }
  }
}
