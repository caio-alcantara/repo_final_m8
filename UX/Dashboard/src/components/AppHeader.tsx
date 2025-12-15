import { BarChart3, FileText, Users, Settings, Clock } from "lucide-react";
import { NavLink } from "@/components/NavLink";
import inteliLogo from "@/assets/inteli-logo.png";

const AppHeader = () => {
  return (
    <header className="bg-[#2E2640] px-6 py-3">
      <div className="max-w-[1400px] mx-auto flex items-center justify-between">
        {/* Left side - Navigation */}
        <nav className="flex items-center gap-1">
          <NavLink 
            to="/" 
            className="flex items-center gap-2 px-3 py-2 rounded-md text-primary-foreground/70 hover:text-primary-foreground hover:bg-primary/20 transition-colors"
            activeClassName="bg-primary/20 text-primary-foreground"
          >
            <BarChart3 className="w-5 h-5" />
            <span className="text-sm font-medium">Dashboard</span>
          </NavLink>
          
          <NavLink 
            to="/documents" 
            className="flex items-center gap-2 px-3 py-2 rounded-md text-primary-foreground/70 hover:text-primary-foreground hover:bg-primary/20 transition-colors"
            activeClassName="bg-primary/20 text-primary-foreground"
          >
            <FileText className="w-5 h-5" />
            <span className="text-sm font-medium">Documentos</span>
          </NavLink>
          
          <NavLink 
            to="/users" 
            className="flex items-center gap-2 px-3 py-2 rounded-md text-primary-foreground/70 hover:text-primary-foreground hover:bg-primary/20 transition-colors"
            activeClassName="bg-primary/20 text-primary-foreground"
          >
            <Users className="w-5 h-5" />
            <span className="text-sm font-medium">Usuários</span>
          </NavLink>
          
          <NavLink 
            to="/config" 
            className="flex items-center gap-2 px-3 py-2 rounded-md text-primary-foreground/70 hover:text-primary-foreground hover:bg-primary/20 transition-colors"
            activeClassName="bg-primary/20 text-primary-foreground"
          >
            <Settings className="w-5 h-5" />
            <span className="text-sm font-medium">Configurações</span>
          </NavLink>

          {/* Last update indicator */}
          <div className="flex items-center gap-1.5 ml-4 px-2 py-1 text-primary-foreground/60 text-xs">
            <Clock className="w-3.5 h-3.5" />
            <span>Atualizado às 18h02 de 12/11/2025</span>
          </div>
        </nav>

        {/* Right side - Logo */}
        <img src={inteliLogo} alt="Inteli" className="h-7" />
      </div>
    </header>
  );
};

export default AppHeader;
