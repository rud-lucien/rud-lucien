
#=
startingVolume = 50
concentrationFactor = 5
concentrationVolume = startingVolume / concentrationFactor
permiateVolume = startingVolume - concentrationVolume
diafiltrationConstant = 8
totalDiafiltrationVolume = diafiltrationConstant * concentrationVolume
maxStirCellVolume = 50
availableDFVolume = maxStirCellVolume - concentrationVolume
totalNumberofDFSteps = totalDiafiltrationVolume / availableDFVolume
DFVolumeperStep = totalDiafiltrationVolume / totalNumberofDFSteps
=#
const diafiltration_constant = 8.0

function calculate_parameters(starting_volume, concentration_factor, max_stir_cell_volume)
    concentration_volume = starting_volume / concentration_factor
    permiate_volume = starting_volume - concentration_volume
    total_diafiltration_volume = diafiltration_constant * concentration_volume
    available_df_volume = max_stir_cell_volume - concentration_volume
    total_number_of_df_steps = ceil(total_diafiltration_volume / available_df_volume)
    df_volume_per_step = total_diafiltration_volume / total_number_of_df_steps

    return (
        concentration_volume = concentration_volume,
        permiate_volume = permiate_volume,
        total_diafiltration_volume = total_diafiltration_volume,
        total_number_of_df_steps = total_number_of_df_steps,
        df_volume_per_step = df_volume_per_step,
    )
end

starting_volume = 50
concentration_factor = 5
max_stir_cell_volume = 50

results = calculate_parameters(starting_volume, concentration_factor, max_stir_cell_volume)

println("Concentration Volume: ", results.concentration_volume)
println("Permeate Volume: ", results.permiate_volume)
println("Total Diafiltration Volume: ", results.total_diafiltration_volume)
println("Total Number of Diafiltration Steps: ", results.total_number_of_df_steps)
println("Diafiltration Volume per Step: ", results.df_volume_per_step)
