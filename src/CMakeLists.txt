# add each subdirectory
foreach (RSLIBS rsRobots rsXML rsScene rsSim)
	add_subdirectory (${RSLIBS})
endforeach ()

