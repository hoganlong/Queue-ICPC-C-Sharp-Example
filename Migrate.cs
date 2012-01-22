// A sample player implemented in C#, based on the Java Migrate
// player.  In this player, each pusher takes one of the red markers
// and tries to gradually move it to vertices that are on the
// boundary between red and non-red.
//
// This is probably only the 5th or 6th C# program I've written, so feel free to
// let me know if there are better ways to use the langauge.
//
// ICPC Challenge
// Sturgill, NC State University

using System;
using System.Collections.Generic;

// Simple representation for a vertex of the map.
public class Vertex3D {
  public int x, y, z;

  // make a 3D vertex with the given coordinates
  public Vertex3D( int vx = 0, int vy = 0, int vz = 0 ) {
    x = vx;
    y = vy;
    z = vz;
  }
};

// Simple representation of a 2D poin/vector.  C# may already
// provide something like this.  If so, I should have used it
// instead.
public class Vector2D {
  public double x, y;

  // Make a 2D point/vector with the given coordinates.
  public Vector2D( double vx = 0, double vy = 0 ) {
    x = vx;
    y = vy;
  }

  // Return the squared magnitude of this vector.
  public double SquaredMag() {
    return x * x + y * y;
  }

  // Return the magnitude of this vector
  public double Mag() {
    return Math.Sqrt( x * x + y * y );
  }

  // Return a unit vector pointing in the same direction as this.
  public Vector2D Norm() {
    double m = Mag();
    return new Vector2D( x / m, y / m );
  }

  // Return a CCW perpendicular to this vector.
  public Vector2D Perp() {
    return new Vector2D( -y, x );
  }

  // Return a new vector thats thisvector rotated by r CCW.
  public Vector2D Rotate( double r ) {
    double s = Math.Sin( r );
    double c = Math.Cos( r );
    return new Vector2D( x * c - y * s,
                         x * s + y * c );
  }

  // Return a cross product of this and b.
  public double Cross( Vector2D b ) {
    return x * b.y - y * b.x;
  }

  // Return a vector pointing in the same direction as this, but with
  // magnitude no greater than d.
  public Vector2D Limit( double d ) {
    double m = Mag();
    if ( m > d )
      return new Vector2D( d * x / m, d * y / m );
    else
      return new Vector2D( x, y );
  }

  // Vector addition.
  public static Vector2D operator +( Vector2D a, Vector2D b ) {
    return new Vector2D( a.x + b.x, a.y + b.y );
  }

  // Vector subtraction.
  public static Vector2D operator -( Vector2D a, Vector2D b ) {
    return new Vector2D( a.x - b.x, a.y - b.y );
  }

  // Return a copy of vector a, scaled by b
  public static Vector2D operator *( Vector2D a, double b ) {
    return new Vector2D( a.x * b, a.y * b );
  }

  // Return a copy of vector a, scaled by b
  public static Vector2D operator *( double b, Vector2D a ) {
    return new Vector2D( b * a.x, b * a.y );
  }

  // Return the dot product of a and b
  public static double operator *( Vector2D a, Vector2D b ) {
    return a.x * b.x + a.y * b.y;
  }
};

public class Migrate {
  // Width and height of the world in game units.
  const int FIELD_SIZE = 100;
  
  // Number of pushers per side.
  const int PCOUNT = 3;

  // Radius of the pusher.
  const double PUSHER_RADIUS = 1;

  // Mass of the pusher.
  const double PUSHER_MASS = 1;

  // Maximum velocity for a pusher
  const double PUSHER_SPEED_LIMIT = 6.0;
  
  // Maximum acceleration for a pusher
  const double PUSHER_ACCEL_LIMIT = 2.0;
  
  // Total number of markers on the field
  const int MCOUNT = 22;

  // Radius of the marker
  const double MARKER_RADIUS = 2;

  // Mass of the marker.
  const double MARKER_MASS = 3;

  // Marker velocity lost per turn
  const double MARKER_FRICTION = 0.35;

  // Width and height of the home region.
  const int HOME_SIZE = 20;

  // Color value for the red player.
  const int RED = 0;
  
  // Color value for the blue player.
  const int BLUE = 1;
  
  // Color value for unclaimed pucks.
  const int GREY = 2;

  // Source of Randomness
  private Random rnd = new Random();

  // Current game score, for red and blue
  private int[] score = new int [ 2 ];

  // Simple representation for a pusher.
  class Pusher {
    // Position of the pusher.
    public Vector2D pos = new Vector2D();
    
    // Pusher velocity
    public Vector2D vel = new Vector2D();
    
    // True if this pusher has a job.
    public bool busy;
    
    // How long we've been doing the current job.  If
    // this number gets to large, we'll pick a new job.
    public int jobTime;
    
    // Target vertex for this pusher.
    public int targetVertex;
    
    public Pusher() {
      busy = false;
    }
  };

  // Simple representation for a marker.
  class Marker {
    // Position of the marker.
    public Vector2D pos = new Vector2D();

    // Marker velocity
    public Vector2D vel = new Vector2D();

    // Marker color
    public int color;
  };

  // Return the value of a, clamped to the [ b, c ] range
  private double Clamp( double a, double b, double c ) {
    if ( a < b )
      return b;
    if ( a > c )
      return c;
    return a;
  }

  /* One dimensional function to help compute acceleration
     vectors. Return an acceleration that can be applied to a pusher
     at pos and moving with velocity vel to get it to target.  The
     alim parameter puts a limit on the acceleration available.  This
     function is used by the two-dimensional MoveTo function to
     compute an acceleration vector toward the target after movement
     perp to the target direction has been cancelled out. */
  private double MoveTo( double pos, double vel, double target,
                         double alim ) {
    // Compute how far pos has to go to hit target.
    double dist = target - pos;

    // Kill velocity if we are close enough.
    if ( Math.Abs( dist ) < 0.01 )
      return Clamp( -vel, -alim, alim );
    
    // How many steps, at minimum, would cover the remaining distance
    // and then stop.
    double steps = Math.Ceiling(( -1 + Math.Sqrt(1 + 8.0 * Math.Abs(dist) / alim)) 
                             / 2.0);
    if ( steps < 1 )
      steps = 1;
    
    // How much acceleration would we need to apply at each step to
    // cover dist.
    double accel = 2 * dist / ( ( steps + 1 ) * steps );
    
    // Ideally, how fast would we be going now
    double ivel = accel * steps;

    // Return the best change in velocity to get vel to ivel.
    return Clamp( ivel - vel, -alim, alim );
  }

  /* Print out a force vector that will move the given pusher to
     the given target location. */
  private void MoveTo( Pusher p, Vector2D target ) {
    // Compute a frame with axis a1 pointing at the target.
    Vector2D a1, a2;

    // Build a frame (a trivial one if we're already too close).
    double dist = ( target - p.pos ).Mag();
    if ( dist < 0.0001 ) {
      a1 = new Vector2D( 1.0, 0.0 );
      a2 = new Vector2D( 0.0, 1.0 );
    } else {
      a1 = ( target - p.pos ) * ( 1.0 / dist );
      a2 = a1.Perp();
    }
        
    // Represent the pusher velocity WRT that frame.
    double v1 = a1 * p.vel;
    double v2 = a2 * p.vel;

    // Compute a force vector in this frame, first cancel out velocity
    // perp to the target.
    double f1 = 0;
    double f2 = -v2;

    // If we have remaining force to spend, use it to move toward the target.
    if ( Math.Abs( f2 ) < PUSHER_ACCEL_LIMIT ) {
      double raccel = Math.Sqrt( PUSHER_ACCEL_LIMIT * PUSHER_ACCEL_LIMIT - 
                                 v2 * v2 );
      f1 = MoveTo( -dist, v1, 0.0, raccel );
    }

    // Convert force 
    Vector2D force = a1 * f1 + a2 * f2;
    Console.Write( force.x + " " + force.y );
  }

  /* Print out a force vector that will move the given pusher around
     to the side of marker m that's opposite from target.  Return true
     if we're alreay behind the marker.  */
  private bool MoveAround( Pusher p, Marker m, Vector2D target ) {
    // Compute vectors pointing from marker-to-target and Marker-to-pusher
    Vector2D mToT = ( target - m.pos ).Norm();
    Vector2D mToP = ( p.pos - m.pos ).Norm();
    
    // See if we're already close to behind the marker.
    if ( mToT * mToP < -0.8 )
      return true;

    // Figure out how far around the target we need to go, we're
    // going to move around a little bit at a time so we don't hit
    // the target.
    double moveAngle = Math.Acos( mToT * mToP );
    if ( moveAngle > Math.PI * 0.25 )
      moveAngle = Math.PI * 0.25;

    // We're not, decide which way to go around.
    if ( mToT.Cross( mToP ) > 0 ) {
      // Try to go around to the right.
      MoveTo( p, m.pos + mToP.Rotate( moveAngle ) * 4.0 );
    } else {
      // Try to go around to the left.
      MoveTo( p, m.pos + mToP.Rotate( -moveAngle ) * 4.0 );
    }

    return false;
  }

  public static void Main() {
    // Make an instance of the player, and let it play the gaqme.
    Migrate migrate = new Migrate();
    migrate.Run();
  }

  private void Run() {
    // Read the static parts of the map.

    // Read the list of vertex locations.
    int n = int.Parse( Console.ReadLine() );

    // List of points in the map.
    Vertex3D[] vertexList = new Vertex3D [ n ];
    for ( int i = 0; i < n; i++ ) {
      string[] tokens = Console.ReadLine().Split();
      vertexList[ i ] = new Vertex3D( int.Parse( tokens[ 0 ] ),
                                      int.Parse( tokens[ 1 ] ),
                                      int.Parse( tokens[ 2 ] ) );
    }

    // Read the list of region outlines.
    n = int.Parse( Console.ReadLine() );
    // List of regions in the map
    int[][] regionList = new int [ n ] [];
    for ( int i = 0; i < n; i++ ) {
      string[] tokens = Console.ReadLine().Split();
      int m = int.Parse( tokens[ 0 ] );
      regionList[ i ] = new int [ m ];
      for ( int j = 0; j < m; j++ )
        regionList[ i ][ j ] = int.Parse( tokens[ j + 1 ] );
    }

    // List of current region colors, pusher and marker locations.
    // These are updated on every turn snapshot from the game.
    int[] regionColors = new int [ regionList.Length ];
    Pusher[] pList = new Pusher [ 2 * PCOUNT ];
    for ( int i = 0; i < pList.Length; i++ )
      pList[ i ] = new Pusher();
    Marker[] mList = new Marker [ MCOUNT ];
    for ( int i = 0; i < mList.Length; i++ )
      mList[ i ] = new Marker();

    int turnNum = int.Parse( Console.ReadLine() );
    while ( turnNum >= 0 ) {
      string[] tokens = Console.ReadLine().Split();
      score[ RED ] = int.Parse( tokens[ 0 ] );
      score[ BLUE ] = int.Parse( tokens[ 1 ] );

      // Read all the region colors.
      tokens = Console.ReadLine().Split();
      n = int.Parse( tokens[ 0 ] );
      for ( int i = 0; i < regionList.Length; i++ )
        regionColors[ i ] = int.Parse( tokens[ i + 1 ] );

      // Read all the pusher locations.
      n = int.Parse( Console.ReadLine() );
      for ( int i = 0; i < pList.Length; i++ ) {
        tokens = Console.ReadLine().Split();
        pList[ i ].pos.x = double.Parse( tokens[ 0 ] );
        pList[ i ].pos.y = double.Parse( tokens[ 1 ] );
        pList[ i ].vel.x = double.Parse( tokens[ 2 ] );
        pList[ i ].vel.y = double.Parse( tokens[ 3 ] );
      }

      // Read all the marker locations.
      n = int.Parse( Console.ReadLine() );
      for ( int i = 0; i < n; i++ ) {
        tokens = Console.ReadLine().Split();
        mList[ i ].pos.x = double.Parse( tokens[ 0 ] );
        mList[ i ].pos.y = double.Parse( tokens[ 1 ] );
        mList[ i ].vel.x = double.Parse( tokens[ 2 ] );
        mList[ i ].vel.y = double.Parse( tokens[ 3 ] );
        mList[ i ].color = int.Parse( tokens[ 4 ] );
      }
    
      // Compute a bit vector for the region colors incident on each
      // vertex.
      int[] vertexColors = new int [ vertexList.Length ];
      for ( int i = 0; i < regionList.Length; i++ )
        for ( int j = 0; j < regionList[ i ].Length; j++ )
          vertexColors[ regionList[ i ][ j ] ] |= ( 1 << regionColors[ i ] );
      
      // Candidate vertices for putting a marker on, vertices that have
      // some red but are not all red.
      List< int > candidates = new List< int >();
      for ( int i = 0; i < vertexList.Length; i++ )
        if ( ( vertexColors[ i ] & 0x1 ) == 1 &&
             vertexColors[ i ] != 1  )
          candidates.Add( i );

      // Choose a next action for each pusher, each pusher is responsible
      // for the marker with the same index.
      for ( int pdex = 0; pdex < PCOUNT; pdex++ ) {
        Pusher p = pList[ pdex ];
      
        // See how long this pusher has been doing its job.
        if ( p.busy ) {
          // Go to idle if we work to long on the same job.
          p.jobTime++;
          if ( p.jobTime >= 60 )
            p.busy = false;
        }

        // If we lose our marker, then just sit idle.
        if ( mList[ pdex ].color != RED ) {
          p.busy = false;
        }
        
        // Otherwise, try to find a new place to push our marker.
        if ( mList[ pdex ].color == RED &&
             !p.busy ) {
          if ( candidates.Count > 0 ) {
            int choice = rnd.Next( candidates.Count );
            p.targetVertex = candidates[ choice ];
            candidates.RemoveAt( choice );
            p.busy = true;
          }
        }

        // Choose a move direction in support of our current goal.
        if ( p.busy ) {
          // Get behind our marker and push it toward its destination.
          Vertex3D v = vertexList[ p.targetVertex ];
          Vector2D dest = new Vector2D( v.x, v.y );
          if ( MoveAround( p, mList[ pdex ], dest ) ) {
            Vector2D mToD = ( dest - mList[ pdex ].pos ).Norm();
            MoveTo( p, mList[ pdex ].pos - mToD );
          }
        } else
          Console.Write( "0.0 0.0" );

        // Print a space or a newline depending on whether we're at
        // the last pusher.
        if ( pdex + 1 < PCOUNT )
          Console.Write( " " );
        else
          Console.WriteLine();
      }

      turnNum = int.Parse( Console.ReadLine() );
    }
  }
}
