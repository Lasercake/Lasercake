/*

    Copyright Eli Dupree and Isaac Dupree, 2011, 2012, 2013

    This file is part of Lasercake.

    Lasercake is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    Lasercake is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with Lasercake.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "gl_rendering.hpp"
#include "main.hpp"
#include "gl_data_format.hpp"

// Hopefully someday this file will be unnecessary.

void gl_renderer::render_2d_text_overlay_(
    abstract_gl_data const& abstract_gl_data,
    viewport_dimension viewport_width,
    viewport_dimension viewport_height,
    LasercakeGLWidget& gl_widget
) {
  gl_data_format::gl_all_data const& gl_data = abstract_gl_data.data();
  QPainter painter(&gl_widget);
  painter.setRenderHint(QPainter::Antialiasing);
  const QString text = QString::fromUtf8(gl_data.hud_text.text.c_str());
  painter.setOpacity(gl_data.hud_text.c.a / 255.0);
  painter.setPen(QColor(gl_data.hud_text.c.r, gl_data.hud_text.c.g, gl_data.hud_text.c.b));
  painter.setFont(QFont(gl_data.hud_text.font_name.c_str(), gl_data.hud_text.point_size));
  painter.drawText(
    gl_data.hud_text.horizontal_margin_in_pixels,
    gl_data.hud_text.vertical_margin_in_pixels,
    viewport_width - 2*gl_data.hud_text.horizontal_margin_in_pixels,
    viewport_height - 2*gl_data.hud_text.vertical_margin_in_pixels,
    Qt::AlignBottom | Qt::AlignHCenter | Qt::TextWordWrap,
    text);
  painter.end();
}
