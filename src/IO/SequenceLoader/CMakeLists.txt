# add only support dependant includes
# ${RADIUM_SRC_DIR} is always added by the parent

file( GLOB_RECURSE sequence_sources *.cpp *.c )
file( GLOB_RECURSE sequence_headers *.hpp *.h )
file( GLOB_RECURSE sequence_inlines *.inl )

set( sequence_sources ${sequence_sources} ${sequencelib_sources} PARENT_SCOPE )
set( sequence_headers ${sequence_headers} PARENT_SCOPE )
set( sequence_inlines ${sequence_inlines} PARENT_SCOPE )

set( RADIUM_IO_IS_INTERFACE FALSE PARENT_SCOPE )
