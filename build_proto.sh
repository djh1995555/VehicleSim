pwd=$(pwd -P)
lwd=$(dirname $pwd)

proto_dir=$pwd/proto
echo $proto_dir
for item in "$proto_dir"/*; do
    echo $item
    filename=$(basename $item)
    # if [ "$filename" == *.proto]; then
    #     protoc $item  --python_out=./ 
    # fi
    if [[ $filename =~ \.proto$ ]]; then
        protoc -I $proto_dir $item  --python_out=$proto_dir
    fi
done

