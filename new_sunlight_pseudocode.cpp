

struct tile_face {
  tile_location tile;
  cardinal_direction which_face;
}

struct tile_edge {
  tile_location tile;
  something_or_other which_edge;
}

struct tile_face_shadow_info {
  collection<tile_face> faces_directly_shadowing_this;
  moving_bbox_collision_detector<1> x_axis_edges;
  moving_bbox_collision_detector<1> y_axis_edges;
  moving_bbox_collision_detector<1> sun_angle_edges;
  
  something_or_other revealed_shape;
  
  tile_face adj_faces[4]; // not sure how these should be arranged
}

enum shadow_change_type {
  COLLISION, // shadow edges facing opposite directions collide
  SUBSUMATION, // a shadow edge catches up to another facing the same direction: note that this will also trigger a PROCESSION at a higher tile
  SEPARATION, // shadow edges facing opposite directions separate: note that this will also trigger a RECESSION at a higher tile which will shed light on this one.
  PROCESSION, // a shadow edge hits the opposite edge of the tile_face
  RECESSION, // a shadow edge recedes past an edge of the tile face
}

struct shadow_change {
  tile_face face;
  shadow_change_type type;
  tile_face shadow1;
  tile_face shadow2; // if any
  
  dimension_identifier which_dimension (maybe?);
}

struct lighting_distribution {
public:
  void do_shadow_change(time t, shadow_change c) {
    tile_face_shadow_info& inf = shadow_info[c.face];
    if (c.shadow1 is no longer in inf)) return;
  
    if (c.type == COLLISION) {
      if (c.shadow2 is no longer in inf)) return;
      if (c.shadow1 and c.shadow2 do not intersect on c.face at time t) return; // TODO evaluate the need for exactitude
      
      // do stuff
    }
    else if (c.type == SUBSUMATION) {
      if (c.shadow2 is no longer in inf)) return;
      if (neither shadow subsumes the other at time t) return; // TODO evaluate the need for exactitude
      
      inf.erase(the one subsumed);
      inf.record_next_possible_change(the one not subsumed);
    }
    else if (c.type == PROCESSION) {
      // No extra checks - procession only fails if the shadow is no longer on the face at all.
      
      tile_face_shadow_info* inf_next = find_as_pointer(shadow_info, inf.adj_faces[the direction we are proceeding]]);
      inf_next.insert(c.shadow1, shadow_changes); // insert automatically calls record_next_possible_change
      
      for each face f2 shadowed by c.face {
        shadow_info[f2].insert(c.shadow1, shadow_changes);
      }
    }
    else if (c.type == RECESSION) {
      // No extra checks - recession only fails if the shadow is no longer on the face at all.
      
      inf.erase(c.shadow1);
      tile_face_shadow_info* inf_next = find_as_pointer(shadow_info, inf.adj_faces[the direction we are receding]]);
      if (inf_next is actually facing a direction on which light would fall) {
        if (inf_next not in shadow_info) {
          create inf_next and ...
          I'm not sure exactly how to do this
          inf_next definitely needs c.shadow1 inserted into it
          but it also probably needs other shadows
        }
      }
      else {
        do a laser-like search for face(s) on which light should now fall
        for each one that's not in shadow_info {
          do (approximately?) the same thing as above
        }
      }
    }
    else { assert(false); }
    
    // logically we only need to do this check for COLLISION and PROCESSION
    if (inf.is_definitely_completely_obscured()) shadow_info.erase(c.face);
  }
  void advance_to_time(time t) {
    while (the first of shadow_changes is before t) {
      do_shadow_change (pop the first of shadow_changes)
    }
  }
private:
  heap<time, shadow_change> shadow_changes;
  unordered_map<tile_face, tile_shadow_info> shadow_info;
}
