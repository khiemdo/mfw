#!/bin/bash

# --- DOCU ---
# This script makes the doxygen docu.

# script configs/variables
DOCU_OUTPUT_DIR=output
DOXYGEN_LOG_FILE=doxygen.log
INDEX_HTML_FILE=index.html
CLOC_OUTPUT_FILE=statistics.md
CLOC_INPUT_PATH=../../..
CLOC_EXCLUDE_LIST_FILE=../../cloc-exclude-list-file.txt
CLOC_DEF_LANG_FILE=

function create_index_html # param none
{
  touch $INDEX_HTML_FILE
	echo -e \
        "<html>\n" \
        "  <head>\n" \
        "    <meta http-equiv=\"refresh\" content=\"0; url=html/index.html\" />\n" \
        "  </head>\n" \
        "</html>\n" > $INDEX_HTML_FILE;
}

echo "Make mfw docu... "

if [ ! -d "$DOCU_OUTPUT_DIR" ]; then
  mkdir DOCU_OUTPUT_DIR
fi
echo "Docu output directory is: $DOCU_OUTPUT_DIR"


echo -n "Create cloc docu page... "
cd doc-tools/cloc
./run-cloc-to-doxy.sh $CLOC_INPUT_PATH $CLOC_EXCLUDE_LIST_FILE $CLOC_DEF_LANG_FILE >../../output/$CLOC_OUTPUT_FILE
cd ../..
echo "OK."

echo -n "Getting git tag... "
export PROJECT_NUMBER="$(git tag -l --contains HEAD)"
if [[ -z  $PROJECT_NUMBER  ]]
then
  echo "Fail. You are not on a tagged commit! Using SHA-ID for doxygen version string instead."
  echo -n "Getting git SHA-ID... "
  export PROJECT_NUMBER="$(echo -n 'SHA-ID: ' ; git rev-parse HEAD ; git diff-index --quiet HEAD || echo '(with uncommitted changes)')"
  echo "OK."
else
  echo "OK."
fi

echo -n "Run doxygen, redirecting output to $DOXYGEN_LOG_FILE ... "
doxygen mfw.doxyfile > $DOXYGEN_LOG_FILE 2>&1
echo "OK."

echo -n "Create file $DOCU_OUTPUT_DIR/$INDEX_HTML_FILE... "
cd output
create_index_html
cd ..
echo "OK."
