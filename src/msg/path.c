/**
 * @file path.c
 * @date 2013/10/02 - last modified 2013/10/03
 */
#include <sns/path.h>


/**
 * @function sns_msg_path_dense_alloc
 * @brief Returns a path msg with enough space allocated for n_steps and n_dof
 */
struct sns_msg_path_dense *sns_msg_path_dense_alloc( uint32_t _n_steps,
						     uint32_t _n_dof ) {
  size_t size = sns_msg_path_dense_size( &(struct sns_msg_path_dense){
      .n_dof = _n_dof, .n_steps = _n_steps } );
  struct sns_msg_path_dense *msg = (struct sns_msg_path_dense*) malloc( size );
  memset( msg, 0, sizeof(*msg) );

  msg->n_dof = _n_dof;
  msg->n_steps = _n_steps;

  return msg;
}

/**
 * @function sns_path_dense_dump
 * @brief Outputs the path to a file (i.e. STDOUT) in a readable form
 */
void sns_path_dense_dump( FILE* _out,
			  const struct sns_msg_path_dense *_msg ) {
  //dump_header( _out, &_msg->header, "path_ref" );

  uint32_t n_dof = _msg->n_dof;
  uint32_t n_steps = _msg->n_steps;
  double t0 = _msg->t0;
  double period = _msg->period;

  fprintf( _out, "n_dof: %d \n", n_dof );
  fprintf( _out, "n_steps: %d \n", n_steps );
  fprintf( _out, "t0: %lf \n", t0 );
  fprintf( _out, "period: %lf \n", period );


  uint32_t i, j;
  for( i = 0; i < _msg->n_steps; i++ ) {
    for( j = 0; j < n_dof; ++j ) {
      fprintf( _out, "\t %f", _msg->x[i*n_dof + j]);
    }
    fprintf( _out, "\n" );
  }
  
}


