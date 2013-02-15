

struct tile_face {
  tile_location tile;
  cardinal_direction which_face;
}

struct tile_edge {
  tile_location tile;
  something_or_other which_edge;
}

struct tile_face_shadow_info {
  collection<tile_edge> edges_directly_shadowing_this; // TODO do we need this? redundant with lit_shape

  // dunno if these will actually use the moving_bbox_collision_detector type
  moving_bbox_collision_detector<1> x_axis_edges;
  moving_bbox_collision_detector<1> y_axis_edges;
  moving_bbox_collision_detector<1> sun_angle_edges;

  // This structure is one of the cruces of the shadow edges algorithm.
  // It is a collection of loops of shadow edges or the edges of this face,
  //     the interior of which is an illuminated area.
  // However, because the visual positions of the edges are continuously changing,
  //     we don't actually record the visual positions of the edges on the face!
  // "What?" you say?
  // The key is that the TOPOLOGICAL structure does NOT change except at certain identifiable moments,
  //     which are handled by the shadow_changes system.
  // The lists need to be doubly linked, and the collection needs to have a way to look up
  //    the instances of a particular edge in log-or-better time
  //    (note that there can be more than one instance of an edge, although it's rare,
  //      as it only happens in a subset of the times when there is more than one loop,
  //      which is rare.)
  // They probably will be something fancier than just doubly linked list structures.
  (collection of doubly linked lists (maybe) of tile_edges) lit_shape;
  
  TODO we need to handle the case where e.g. when a moving diagonal line is pinched off by two axis aligned lines.


  [a bunch of insertion and deletion functions]
    most of which need to have access to the shadow_changes heap so that they can record the next-collisions of anything that was inserted,
    and faces_by_shadow_edge so they can record... hmm, maybe these should be private functions of lighting_distribution instead.
  
  
  tile_face adj_faces[4]; // not sure how these should be arranged / how we can make use of them
}

enum shadow_change_type {
  COLLISION, // shadow edges facing opposite directions collide
  SUBSUMATION, // a shadow edge catches up to another facing the same direction.
  PROCESSION, // a shadow edge hits the opposite edge of the tile_face
  RECESSION, // a shadow edge recedes past an edge of the tile face
  // shadow edges will probably be recorded in a way that has direction, so COLLISION/SUBSUMATION and PROCESSION/RECESSION could be combined into each other.
  // Actually we could just make the edge of the tile be the second tile_edge recorded in shadow_change, eliminating the need for types at all.
}

struct shadow_change {
  tile_face face;
  shadow_change_type type;
  tile_edge shadow1;
  tile_edge shadow2; // if any.
    // Standardization: SUBSUMATION is standardized as "shadow2, moving slower, subsumes shadow1, which is moving faster"
    // A faster-moving shadow is also always closer to the sun;
    // the standard is that shadow1 is closer to the sun.

  // Unnecessary because we now use tile_edges, which include dimension.
  // dimension_identifier which_dimension (maybe?);
}

struct lighting_distribution {
public:
  void do_shadow_change(time t, shadow_change c) {
    tile_face_shadow_info& inf = shadow_info[c.face];
    if (c.shadow1 is no longer in inf)) return;
  
    if (c.type == COLLISION) {
      if (c.shadow2 is no longer in inf)) return;
      assert(c.shadow1 and c.shadow2 are parallel);
      // Assume that shadow1 and shadow2 are at the same (direction-perpendicular-to-them) position.
      // If the time units are exact, this will be true, but I think we're okay with them being inexact (TODO check that mathematically)
      if (c.shadow1 and c.shadow2 don't overlap in the direction parallel to them) return;
      
      inf.lit_shape.do something at the point of collision (TODO)
      and update other inf members in accordance with that
      it's possible that both edges disappear, or that one of them does, or that neither does.

      // shadow2 is further from the sun, so shadow1 is now cast there (but not vice versa)
      c.shadow2 (considered as an edge in real space) is an edge of two faces, one of which is lit and the other of which is not.
      tile_face_shadow_info inf2 = shadow_info[the lit face];
      inf2.insert (as a dark area) c.shadow1 and any necessary adjacent edges
    }
    else if (c.type == SUBSUMATION) {
      if (c.shadow2 is no longer in inf)) return;
      if (c.shadow1 is not subsumed at time t) return; // TODO how do I compute this? and do I need times to be exact in order to do so?

      inf.subsume c.shadow1 into c.shadow2;
      inf.record_next_possible_change(c.shadow2);

      // A subsumation is always when a closer-to-the-sun object (shadow1's) stops shadowing
      //    anything beyond what a farther-from-the-sun object (shadow2's) is shadowing.
      // Hence, it's accompanied by (an infinitesimal duration later)
      //    shadow1's object ceasing to shadow some part of shadow2's object.
      c.shadow2 (considered as an edge in real space) is an edge of two faces, one of which is at an angle which can be lit by the sun and the other of which is not.
      tile_face_shadow_info inf2 = shadow_info[the potentially-lit face] (construct it (as totally-dark) if necessary);
      inf2.insert (as a lit area) c.shadow1 and any necessary adjacent edges
    }
    else if (c.type == PROCESSION) {
      // No extra checks - procession only fails if the shadow is no longer on the face at all.
      tile_edge e2 = the edge we're walking over;

      if (e2 joins this face with a parallel face or with one that folds back *towards* the sun) {
        tile_face_shadow_info* inf2 = find_as_pointer(shadow_info, that next face);
        inf2.insert (as a dark area) c.shadow1 and any necessary adjacent edges
      }
      else {
        for (each face f2 with e2 casting an edge of its lit area) {
          shadow_info[f2].insert (as a dark area) c.shadow1 and any necessary adjacent edges
        }
      }
    }
    else if (c.type == RECESSION) {
      // No extra checks - recession only fails if the shadow is no longer on the face at all.
      tile_edge e2 = the edge we're walking over;
      
      inf.subsume shadow1 into e2;

      if (e2 joins this face with a parallel face or with one that folds back *away from* the sun) {
        tile_face_shadow_info& inf2 = shadow_info[the potentially-lit face] (construct it (as totally-dark) if necessary);
        inf2.insert (as a lit area) c.shadow1 and any necessary adjacent edges
      }
      else {
        // e2 is at least partially in shadow, but it might be nonfully in shadow and still shadowing something else.
        // If it is, then that shadow is currently forming a corner with something where, once shadow1 recedes off the edge,
        // there will be a pathway of light going into the corner.
        for (each face f2 with e2 casting an edge of its lit area) {
          handle that
        }

        if (the above process didn't handle the full length of (c.shadow1 u e2)) {
          do a laser-like search (extrusion of the unhandled pieces) for face(s) on which light should now fall
            ignoring, I think, ones with an infinitesimal overlap
          for (each found face f2) {
            assert(e2 isn't casting any of the edges of f2's lit area)
            tile_face_shadow_info& inf2 = shadow_info[the newly-lit face] (construct it (as totally-dark) if necessary);
            inf2.insert (as a lit area) c.shadow1 and any necessary adjacent edges
          }
        }
      }
    }
    else { assert(false); }
    
    // logically we only need to do this check for COLLISION and PROCESSION
    if (inf.is_completely_dark()) shadow_info.erase(c.face);
  }
  void advance_to_time(time t) {
    while (the first of shadow_changes is before t) {
      do_shadow_change (pop the first of shadow_changes)
    }
  }
private:
  heap<time, shadow_change> shadow_changes;
  unordered_map<tile_face, tile_shadow_info> shadow_info;
  unordered_multimap<tile_edge, tile_face> faces_by_shadow_edge; // we need to do this reverse lookup sometimes
}
