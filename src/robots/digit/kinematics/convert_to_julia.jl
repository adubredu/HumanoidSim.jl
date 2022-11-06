function convert_to_julia(input_filename, output_filename)
    lines = readlines(input_filename, keep=true)
    lines = lines[55:end]
    to_del = [] 

    for (i, line) in enumerate(lines) 
        if length(line) > 8
            if line[1:8] == "  double"
                push!(to_del, i) 
            end
        end
    end 
    deleteat!(lines, to_del)

    kin = join(lines)
    kin = replace(kin, ";"=>"")
    kin = replace(kin, "]"=>"+1]")
    kin = replace(kin, ".*"=>".0 * ")
    kin = replace(kin, ". +"=>".0 + ")
    kin = replace(kin, "}"=>"end")
    kin = replace(kin, "Cos"=>"cos")
    kin = replace(kin, "Sin"=>"sin")

    fo = open(output_filename, "w")
    write(fo, kin)
    close(fo) 
end
