import AppHeader from "@/components/AppHeader";
import DashboardHeader from "@/components/dashboard/DashboardHeader";
import AlertsLineChart from "@/components/dashboard/AlertsLineChart";
import TopicsBarChart from "@/components/dashboard/TopicsBarChart";
import ToursLineChart from "@/components/dashboard/ToursLineChart";
import CheckpointsBarChart from "@/components/dashboard/CheckpointsBarChart";
import VisitorsProfileChart from "@/components/dashboard/VisitorsProfileChart";

const Index = () => {
  return (
    <div className="h-screen bg-background flex flex-col overflow-hidden">
      <AppHeader />
      <DashboardHeader />
      
      {/* ALTERAÇÃO: Removido 'lg:overflow-hidden' e adicionado 'overflow-y-auto' para permitir scroll sempre que necessário */}
      <main className="flex-1 w-full overflow-y-auto">
        <div className="max-w-[1400px] w-full mx-auto px-6 py-6">
          
          {/* ALTERAÇÃO: Removido 'h-full' daqui para permitir que o conteúdo cresça e gere scroll */}
          <div className="flex flex-col lg:flex-row gap-4">
            
            {/* Coluna principal - 4 charts */}
            {/* ALTERAÇÃO: Removido 'h-full' para não forçar o tamanho da tela */}
            <div className="lg:w-2/3 grid grid-cols-1 md:grid-cols-2 gap-3">
              <AlertsLineChart />
              <TopicsBarChart />
              <ToursLineChart />
              <CheckpointsBarChart />
            </div>
            
            {/* Coluna lateral - Perfil dos visitantes */}
            <div className="lg:w-1/3 flex flex-col gap-3">
              <h2 className="text-sm font-semibold text-foreground shrink-0">Perfil dos visitantes</h2>
              {/* Adicionado min-h para garantir que o gráfico tenha altura mesmo sem o h-full do pai */}
              <div className="min-h-[400px] lg:h-auto">
                <VisitorsProfileChart />
              </div>
            </div>
          </div>
        </div>
      </main>
    </div>
  );
};

export default Index;