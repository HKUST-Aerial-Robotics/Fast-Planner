function _catkin_make()
{
    local cur prev
    cur=${COMP_WORDS[COMP_CWORD]}
    prev=${COMP_WORDS[COMP_CWORD-1]}

    # autocomplete path arguments for -C, --directory, --source, --build
    case $prev in
        -C|--directory|--source|--build)
            _filedir -d
            return 0
            ;;
    esac

    if [[ "$cur" == -DCMAKE_BUILD_TYPE=* ]]; then
        # autocomplete CMake argument CMAKE_BUILD_TYPE with its options
        COMPREPLY=( $( compgen -P "-DCMAKE_BUILD_TYPE=" -W "None Debug Release RelWithDebInfo MinSizeRel" -- "${cur:19}" ) )
    elif [[ "$cur" == -DCATKIN_ENABLE_TESTING=* ]]; then
        # autocomplete catkin argument CATKIN_ENABLE_TESTING with its options
        COMPREPLY=( $( compgen -P "-DCATKIN_ENABLE_TESTING=" -W "0 1" -- "${cur:24}" ) )
    elif [[ "$cur" == -DCATKIN_DEVEL_PREFIX=* || "$cur" == -DCMAKE_INSTALL_PREFIX=* ]]; then
        COMPREPLY=()
    elif [[ "$cur" == -* ]]; then
        local opts="$( _parse_help "$1" )"
        [[ $opts ]] || opts="$( _parse_usage "$1" )"
        if [[ "$cur" == -* ]]; then
            # suggest some common CMake arguments
            opts="$opts -DCATKIN_DEVEL_PREFIX= -DCATKIN_ENABLE_TESTING= -DCMAKE_INSTALL_PREFIX= -DCMAKE_BUILD_TYPE="
        fi
        COMPREPLY=( $( compgen -W "$opts" -- "$cur" ) )
        [[ $COMPREPLY == *= ]] && compopt -o nospace
    else
        # check if custom workspace root has been specified on the command line
        local workspace_dir="."
        for (( i=0; i < ${#COMP_WORDS[@]}; i++ )); do
            if [[ ${COMP_WORDS[i]} == -C || ${COMP_WORDS[i]} == --directory ]]; then
                # eval to expand tilde
                eval workspace_dir=${COMP_WORDS[i+1]}
            fi
        done
        # check if custom build folder has been specified on the command line
        local build_dir="build"
        for (( i=0; i < ${#COMP_WORDS[@]}; i++ )); do
            if [[ ${COMP_WORDS[i]} == --build ]]; then
                # eval to expand tilde
                eval build_dir=${COMP_WORDS[i+1]}
            fi
        done

        # determine location of Makefile
        local makefile_dir
        if [[ "$build_dir" = /* ]]; then
            makefile_dir="$build_dir"
        else
            makefile_dir="$workspace_dir/$build_dir"
        fi
        COMPREPLY=()
        if [ -f "$makefile_dir/Makefile" ]; then
            cur=${COMP_WORDS[COMP_CWORD]}
            COMPREPLY=( $( compgen -W "`make -C $makefile_dir -qp 2>/dev/null | awk -F':' '/^[a-zA-Z0-9][a-zA-Z0-9_\.]*:/ { print $1 }'`" -- $cur ))
        fi
    fi
} &&
complete -F _catkin_make catkin_make
